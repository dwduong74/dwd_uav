#!/usr/bin/env python3
"""Run Gazebo, PX4 1.17, DDS Agent and the ROS delivery stack together."""
import os
import argparse
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time


def main():
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-dir', type=Path, default=Path(os.environ.get('DELIVERY_PX4_DIR', '/home/dwduong74/PX4-Autopilot')))
    parser.add_argument('--assets', type=Path, default=repo / 'build/sitl_assets')
    parser.add_argument('--scene-only', action='store_true', help='Leave Agent and ROS launch to separate terminals')
    parser.add_argument('--table-c', action='store_true', help='Generate the three-package Table C arena')
    parser.add_argument('--course',type=Path,default=repo/'config/table_c.yaml')
    args = parser.parse_args()
    px4 = args.px4_dir.resolve()
    build = px4 / 'build/px4_sitl_default'
    binary = build / 'bin/px4'
    if not binary.exists():
        raise SystemExit(f'PX4 SITL executable missing: {binary}')
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        raise SystemExit('Run sim from a desktop terminal so Gazebo can open its window.')
    assets = args.assets.resolve()
    prepare=[sys.executable,str(repo/'scripts/prepare_sitl.py'),str(px4),'--output',str(assets)]
    if args.table_c: prepare.extend(['--table-c','--course',str(args.course.resolve())])
    subprocess.run(prepare,check=True)
    runtime = repo / 'build/gazebo_runtime'
    runtime.mkdir(parents=True, exist_ok=True)
    runtime_etc = repo / 'build/gazebo_etc'
    if runtime_etc.exists():
        shutil.rmtree(runtime_etc)
    shutil.copytree(build / 'etc', runtime_etc, symlinks=True)
    # Applied after the stock airframe defaults. Gazebo GNSS has no physical
    # receiver delay, and this companion replaces the absent GCS control link.
    post = runtime_etc / 'init.d-posix/airframes/4001_gz_x500.post'
    post.write_text('param set NAV_DLL_ACT 0\nparam set EKF2_GPS_DELAY 0\n'
                    'param set EKF2_ACC_B_NOISE 0.0001\n')
    logs = repo / 'log/gazebo'
    logs.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(GZ_PARTITION='dwd_uav_native', PX4_GZ_STANDALONE='1',
               PX4_SYS_AUTOSTART='4001', PX4_SIM_MODEL='gz_x500_delivery',
               PX4_GZ_WORLD='delivery', PX4_GZ_MODELS=str(assets / 'models'),
               # This companion is the control link; SITL has no QGroundControl.
               PX4_PARAM_NAV_DLL_ACT='0',
               ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID', '0' if args.scene_only else '74'), ROS_LOCALHOST_ONLY='0',
               PX4_UXRCE_DDS_PORT='8888' if args.scene_only else '8894')
    env['GZ_SIM_RESOURCE_PATH'] = ':'.join(map(str, [assets / 'models', assets / 'worlds',
        px4 / 'Tools/simulation/gz/models', px4 / 'Tools/simulation/gz/worlds']))
    env['GZ_SIM_SERVER_CONFIG_PATH'] = str(px4 / 'src/modules/simulation/gz_bridge/server.config')
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(build / 'src/modules/simulation/gz_plugins')
    version = subprocess.check_output(['git', '-C', str(px4), 'describe', '--tags',
                                       '--always'], text=True).strip()
    if version != 'v1.17.0':
        raise SystemExit(f'This stack requires PX4 v1.17.0; checkout reports {version}')
    stack = 'Gazebo + PX4' if args.scene_only else 'Gazebo + PX4 + DDS Agent + ROS delivery'
    print(f'{stack} {version}; domain {env["ROS_DOMAIN_ID"]}.', flush=True)
    print('No mission starts automatically. Wait for flight/camera/payload readiness.', flush=True)
    if args.table_c: print('Start with: ros2 service call /competition/start std_srvs/srv/Trigger {}',flush=True)
    print('Close with Ctrl+C. Logs: ' + str(logs), flush=True)
    children = []
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        commands = [('server', ['gz', 'sim', '-r', '-s', str(assets / 'worlds/delivery.sdf')])]
        gui_enabled = os.environ.get('DELIVERY_HEADLESS') != '1'
        if gui_enabled:
            # Keep visualization responsive enough to observe while reserving
            # CPU scheduling time for PX4, physics and sensor deadlines.
            commands.append(('gui', ['chrt', '--idle', '0', 'gz', 'sim', '-g']))
        commands.append(('px4', [str(binary), '-d', str(runtime_etc), '-w', str(runtime)]))
        if not args.scene_only:
            commands.insert(2, ('agent', ['MicroXRCEAgent', 'udp4', '-p', '8894', '-v', '2']))
            commands.append(('ros', ['ros2', 'launch', 'delivery_ros', 'sitl.launch.py',
                                     'config:=' + str(assets / 'sitl.yaml'),
                                     'course:=' + str(args.course.resolve())]))
        for name, command in commands:
            with (logs / (name + '.log')).open('w') as output:
                child = subprocess.Popen(command, cwd=runtime, env=env, stdout=output,
                                         stderr=subprocess.STDOUT, start_new_session=True)
            children.append((name, child))
        if gui_enabled:
            # The default camera may miss the small vehicle after the GUI
            # restores an old view. Focus the spawned model automatically.
            for _ in range(12):
                result = subprocess.run(['gz', 'service', '-s', '/gui/move_to',
                    '--reqtype', 'gz.msgs.StringMsg', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '500', '--req', 'data: "x500_delivery_0"'],
                    env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if result.returncode == 0:
                    break
                time.sleep(.5)
        while True:
            for name, child in children:
                if child.poll() is not None:
                    print(f'{name} exited ({child.returncode}); see {logs / (name + ".log")}', flush=True)
                    return child.returncode
            time.sleep(.5)
    except KeyboardInterrupt:
        return 0
    finally:
        for name, child in reversed(children):
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for name, child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()


if __name__ == '__main__':
    sys.exit(main())
