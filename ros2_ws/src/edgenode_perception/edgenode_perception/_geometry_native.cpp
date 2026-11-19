#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <utility>
#include <vector>

// Geometry only. No sensor filtering, radii, height rules or safety margins.
static PyObject* component_labels(PyObject*,PyObject* args) {
    Py_ssize_t count;PyObject* object;
    if (!PyArg_ParseTuple(args,"nO",&count,&object)) return nullptr;
    Py_buffer data;
    if(count<0){PyErr_SetString(PyExc_ValueError,"nonnegative point count required");return nullptr;}
    if (PyObject_GetBuffer(object,&data,PyBUF_CONTIG_RO)<0) return nullptr;
    if (data.itemsize!=8 || data.len%16) {
        PyBuffer_Release(&data);PyErr_SetString(PyExc_ValueError,"int64 Nx2 pair buffer required");return nullptr;
    }
    const auto* pairs=static_cast<const int64_t*>(data.buf);
    std::vector<Py_ssize_t> parent(count),rank(count,0),labels(count,-1);
    std::iota(parent.begin(),parent.end(),0);
    auto root=[&](Py_ssize_t i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;};
    for(Py_ssize_t k=0;k<data.len/8;k+=2) {
        auto a=pairs[k],b=pairs[k+1];
        if(a<0 || b<0 || a>=count || b>=count) {
            PyBuffer_Release(&data);PyErr_SetString(PyExc_ValueError,"pair index outside input");return nullptr;
        }
        auto x=root(a),y=root(b);if(x==y)continue;
        if(rank[x]<rank[y])std::swap(x,y);
        parent[y]=x;if(rank[x]==rank[y])++rank[x];
    }
    PyBuffer_Release(&data);PyObject* output=PyList_New(count);if(!output)return nullptr;
    Py_ssize_t next=0;
    for(Py_ssize_t i=0;i<count;++i){auto r=root(i);if(labels[r]<0)labels[r]=next++;PyList_SET_ITEM(output,i,PyLong_FromSsize_t(labels[r]));}
    return output;
}

static PyObject* convex_hull(PyObject*,PyObject* object) {
    Py_buffer data;if(PyObject_GetBuffer(object,&data,PyBUF_CONTIG_RO)<0)return nullptr;
    if(data.itemsize!=8 || data.len%16){PyBuffer_Release(&data);PyErr_SetString(PyExc_ValueError,"float64 Nx2 buffer required");return nullptr;}
    const auto* values=static_cast<const double*>(data.buf);
    using Point=std::pair<double,double>;std::vector<Point> points;
    for(Py_ssize_t i=0;i<data.len/8;i+=2){
        if(!std::isfinite(values[i]) || !std::isfinite(values[i+1])){
            PyBuffer_Release(&data);PyErr_SetString(PyExc_ValueError,"finite observed points required");return nullptr;
        }
        points.emplace_back(values[i],values[i+1]);
    }
    PyBuffer_Release(&data);std::sort(points.begin(),points.end());points.erase(std::unique(points.begin(),points.end()),points.end());
    auto half=[](auto first,auto last){std::vector<Point> h;for(auto i=first;i!=last;++i){
        while(h.size()>1){auto a=h[h.size()-2],b=h.back(),p=*i;
            if((b.first-a.first)*(p.second-b.second)-(b.second-a.second)*(p.first-b.first)>0)break;
            h.pop_back();}
        h.push_back(*i);}return h;};
    std::vector<Point> h;
    if(points.size()>1){auto upper=half(points.begin(),points.end()),lower=half(points.rbegin(),points.rend());upper.pop_back();lower.pop_back();h=std::move(upper);h.insert(h.end(),lower.begin(),lower.end());}else h=points;
    PyObject* output=PyList_New(h.size());if(!output)return nullptr;
    for(size_t i=0;i<h.size();++i)PyList_SET_ITEM(output,i,Py_BuildValue("dd",h[i].first,h[i].second));
    return output;
}
static PyMethodDef methods[]={
    {"component_labels",component_labels,METH_VARARGS,"Exact connected components of all supplied undirected radius pairs."},
    {"convex_hull",convex_hull,METH_O,"Monotone observed hull with the same double arithmetic and ordering as Python."},
    {nullptr,nullptr,0,nullptr}
};
static PyModuleDef definition={PyModuleDef_HEAD_INIT,"_geometry_native",nullptr,-1,methods};
PyMODINIT_FUNC PyInit__geometry_native(){return PyModule_Create(&definition);}
