#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

// Geometry only. No sensor filtering, radii, height rules or safety margins.
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
    {"convex_hull",convex_hull,METH_O,"Monotone observed hull with the same double arithmetic and ordering as Python."},
    {nullptr,nullptr,0,nullptr}
};
static PyModuleDef definition={PyModuleDef_HEAD_INIT,"_geometry_native",nullptr,-1,methods};
PyMODINIT_FUNC PyInit__geometry_native(){return PyModule_Create(&definition);}
