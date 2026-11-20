#!/usr/bin/env python3
"""Install immutable D/original sensor JSON files into MORAI's host save folder.

Run with host Python before launching D. This only copies configuration files;
it never changes the active simulator, connects sensors or commands the car.
Load the printed basename in MORAI Sensor Edit and resume the simulator.
"""
import argparse,hashlib,json
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sensor-save-dir',type=Path,default=Path.home()/'Desktop/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Sensor/26.R1.H3')
    a=p.parse_args();root=Path(__file__).resolve().parents[1];config=root/'config/s4_d'
    baseline=config/'sensors_baseline_20261120.json';dynamic=config/'sensors_d_roof_hdl64_20261120.json'
    b=json.loads(baseline.read_text());d=json.loads(dynamic.read_text())
    assert all(d[k]==v for k,v in b.items() if k!='Lidar3DList')
    assert len(b['Lidar3DList'])==1 and len(d['Lidar3DList'])==2 and d['Lidar3DList'][0]==b['Lidar3DList'][0]
    aux=d['Lidar3DList'][1];assert aux['m_SensorUniqueID']==5 and aux['lc']['ros2NativeConfig']['Topic']=='/lidar_rear/points'
    assert a.sensor_save_dir.is_dir(),'Select the existing MORAI host sensor save directory'
    rows=[]
    for kind,source in [('original',baseline),('d_roof_hdl64',dynamic)]:
        payload=source.read_bytes();digest=hashlib.sha256(payload).hexdigest();target=a.sensor_save_dir/f'edgenode_{kind}_{digest[:12]}.json'
        if target.exists():assert target.read_bytes()==payload,'Refusing to overwrite a different MORAI file'
        else:
            with target.open('xb') as stream:stream.write(payload)
        rows.append(dict(profile=kind,source=str(source),sha256=digest,load_basename=target.name,installed_file=str(target)))
    print(json.dumps(dict(active_simulator_changed=False,original_four_sensors_identical=True,profiles=rows),indent=2))
if __name__=='__main__':main()
