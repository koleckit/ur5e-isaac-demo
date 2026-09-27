"""DoD: exchange messages between two real ROS processes, then stop both."""
import os
import re
import signal
import subprocess
import tempfile
import time
from pathlib import Path

env = os.environ.copy()
env['ROS_DOMAIN_ID'] = '171'
env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
env.pop('ROS_STATIC_PEERS', None)
processes = []
with tempfile.TemporaryDirectory(prefix='ur-ros-dod-') as temp:
    root = Path(temp)
    try:
        with (root / 'talker.log').open('w') as talk_log, (root / 'listener.log').open('w') as listen_log:
            for executable, stream in [('listener', listen_log), ('talker', talk_log)]:
                processes.append(subprocess.Popen(
                    ['ros2', 'run', 'demo_nodes_cpp', executable],
                    env=env, stdout=stream, stderr=subprocess.STDOUT,
                    start_new_session=True))
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('ROS node exited before communication succeeded')
                sent = set(re.findall(r'Hello World: (\d+)', (root / 'talker.log').read_text()))
                received = set(re.findall(r'Hello World: (\d+)', (root / 'listener.log').read_text()))
                matches = sent & received
                if len(matches) >= 3:
                    print(f'PASS: talker/listener exchanged {len(matches)} matching messages; domain=171, localhost')
                    break
                time.sleep(0.25)
            else:
                raise RuntimeError('No three matching messages within 25 seconds')
    finally:
        for p in processes:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        for name in ['talker.log', 'listener.log']:
            target = Path('ros-dod-' + name)
            if (root / name).exists():
                target.write_bytes((root / name).read_bytes())
                print(f'Log: {target}')
