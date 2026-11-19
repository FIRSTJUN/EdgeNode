"""Build all original seeds, retaining surveyed islands as actual holes."""
import hashlib,json,sys
from pathlib import Path
import s4_build_surveyed_roads as builder
from s4_partitioned_surveyed_roads import local_patches
from s4_survey_islands import prepare,records

original=builder.HDMapBoundaries
prepared=[]
def boundary(*args,**kwargs):
    result=prepare(original(*args,**kwargs));prepared.append(result);return result

if __name__=='__main__':
    builder.local_patches=local_patches;builder.HDMapBoundaries=boundary
    builder.main()
    output=Path(sys.argv[sys.argv.index('--output')+1])
    data=json.loads(output.read_text());data['schema']='edgenode.surveyed-local-road.v2'
    data['negative_islands']=records(prepared[0])
    data['source']='Exact lower C3 protection boundaries, endpoint-partitioned local surfaces minus actual closed non-driving enclosures; no width inference or gap closing'
    data['physical_type_authority']='NGII 2019 p104 table68: 2 guardrail,3 concrete barrier,4 curb,5 anti-crossing facility,7 temporary structure,8 wall,99 other vehicle safety facility; 6 median openings explicitly excluded'
    data['algorithm_files_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        [Path(__file__),Path('tools/s4_partitioned_surveyed_roads.py'),Path('tools/s4_survey_islands.py')]}
    final=output.with_name(output.stem+'-v2.json');assert not final.exists();final.write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps(dict(final=str(final),sha256=hashlib.sha256(final.read_bytes()).hexdigest(),negative_islands=len(data['negative_islands']))),flush=True)
