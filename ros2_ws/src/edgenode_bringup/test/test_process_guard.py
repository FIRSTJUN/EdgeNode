from pathlib import Path
from edgenode_bringup.process_guard import existing_autonomy_processes


def process(root, pid, args, domain=0):
    p=root/str(pid);p.mkdir()
    (p/'cmdline').write_bytes(b'\0'.join(a.encode() for a in args)+b'\0')
    (p/'environ').write_bytes(('ROS_DOMAIN_ID='+str(domain)).encode()+b'\0')


def test_module_and_installed_entrypoint_are_detected_but_shell_text_is_not(tmp_path):
    process(tmp_path,10,['/usr/bin/python3','-m','edgenode_control.control_node'])
    process(tmp_path,11,['/usr/bin/python3','/workspace/ros2_ws/install/edgenode_localization/lib/edgenode_localization/localization_node','--ros-args'])
    process(tmp_path,12,['bash','-c','python3 -m edgenode_planning.planning_node'])
    assert {p['node'] for p in existing_autonomy_processes(tmp_path,0,999)}=={'control_node','localization_node'}


def test_other_domains_and_own_process_do_not_block(tmp_path):
    process(tmp_path,10,['python3','-m','edgenode_control.control_node'],193)
    process(tmp_path,11,['python3','-m','edgenode_planning.planning_node'],0)
    assert existing_autonomy_processes(tmp_path,0,11)==[]
