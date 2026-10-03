"""Gracefully stop this workspace's autonomy processes for an explicit restart."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

PACKAGES = ('edgenode_perception', 'edgenode_planning', 'edgenode_control')


def process_kind(args, cwd, root):
    executables = {
        str(root/'ros2_ws/install'/p/'lib'/p/(p.removeprefix('edgenode_')+'_node'))
        for p in PACKAGES
    }
    if any(arg in executables for arg in args):
        return 'node'
    try:
        Path(cwd).relative_to(root)
    except (ValueError, TypeError):
        return None
    for i in range(len(args)-2):
        if args[i:i+3] == ['launch', 'edgenode_bringup', 'autonomy.launch.py']:
            if any(Path(a).name == 'ros2' for a in args[:i]):
                return 'launch'
    return None


def processes(root):
    found = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            status = (entry/'status').read_text()
            if any(line.startswith('State:') and 'Z' in line for line in status.splitlines()):
                continue
            args = (entry/'cmdline').read_bytes().decode(errors='replace').rstrip('\0').split('\0')
            kind = process_kind(args, (entry/'cwd').resolve(), root)
            if kind:
                found[int(entry.name)] = kind
        except (OSError, ValueError):
            continue
    return found


def send(pid, sig):
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        pass


def main():
    root = Path(__file__).resolve().parents[2]
    existing = processes(root)
    if not existing:
        print('No local autonomy pipeline to stop.', flush=True)
        return
    print('Stopping existing EdgeNode pipeline:', ', '.join(map(str, sorted(existing))), flush=True)
    try:
        response = subprocess.run(
            ['ros2', 'param', 'set', '/planning_node', 'enable_drive', 'false'],
            capture_output=True, text=True, timeout=5)
    except subprocess.TimeoutExpired:
        raise SystemExit('Planner did not respond. Restart cancelled; inspect the existing pipeline.')
    if response.returncode != 0 or 'Set parameter successful' not in response.stdout:
        raise SystemExit('Could not disable the existing planner. Restart cancelled: '+
                         (response.stdout+response.stderr).strip())
    # Leave time for the old controller to publish braking before shutdown.
    time.sleep(1.5)
    for pid, kind in existing.items():
        if kind == 'launch':
            send(pid, signal.SIGINT)
    end = time.monotonic()+6
    while time.monotonic() < end:
        remaining = processes(root)
        if not remaining:
            print('Existing pipeline stopped.', flush=True)
            return
        time.sleep(.2)
    # Standalone nodes, or launch children still alive, get their graceful handler.
    for pid, kind in processes(root).items():
        if kind == 'node':
            send(pid, signal.SIGTERM)
    end = time.monotonic()+4
    while time.monotonic() < end:
        if not processes(root):
            print('Existing pipeline stopped.', flush=True)
            return
        time.sleep(.2)
    raise SystemExit('Some old nodes are still running. Restart cancelled; no second controller started.')


if __name__ == '__main__':
    main()
