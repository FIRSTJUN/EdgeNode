"""Prevent a second local autonomy launch in the same ROS domain."""
import os
from pathlib import Path

NODES = ('localization', 'planning', 'control')


def existing_autonomy_processes(proc_root=Path('/proc'), domain_id=None, own_pid=None):
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0') or '0') if domain_id is None else domain_id
    own = os.getpid() if own_pid is None else own_pid
    result = []
    for directory in proc_root.iterdir():
        if not directory.name.isdigit() or int(directory.name) == own:
            continue
        try:
            args = [a.decode(errors='replace') for a in (directory/'cmdline').read_bytes().split(b'\0') if a]
            node = None
            for kind in NODES:
                module = 'edgenode_'+kind+'.'+kind+'_node'
                entry = '/lib/edgenode_'+kind+'/'+kind+'_node'
                if (any(a.endswith(entry) for a in args[:2])
                        or any(a == '-m' and args[i+1:i+2] == [module] for i,a in enumerate(args[:4]))):
                    node = kind+'_node'
                    break
            if node is None:
                continue
            try:
                fields = (directory/'environ').read_bytes().split(b'\0')
                value = next((v.split(b'=',1)[1] for v in fields if v.startswith(b'ROS_DOMAIN_ID=')), b'0')
                other_domain = int(value or b'0')
            except (OSError, ValueError):
                other_domain = domain  # Unknown domain cannot authorize a duplicate.
            if other_domain == domain:
                result.append(dict(pid=int(directory.name), node=node, domain=domain))
        except (OSError, ProcessLookupError):
            continue
    return result


def assert_no_existing_autonomy():
    processes = existing_autonomy_processes()
    if processes:
        detail = ', '.join(p['node']+'(pid='+str(p['pid'])+')' for p in processes)
        raise RuntimeError('Autonomy is already running in this ROS domain: '+detail+
                           '. Keep the existing nodes; do not start a second autonomy launch.')
