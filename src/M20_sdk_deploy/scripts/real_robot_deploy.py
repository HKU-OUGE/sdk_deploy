#!/usr/bin/env python3
"""Start/stop the M20 real-robot deployment from the local computer.

Default network topology:
  local PC -> 103 WiFi host: user@10.21.41.1
  local PC -> 106 perception host through 103: user@10.21.33.106

The script starts only this repo's deployment processes.  It leaves vendor
processes under /opt/robot running, except for restarting LIO/height-map
services when requested by the start sequence.
"""

import argparse
import base64
import datetime as _dt
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
LOCAL_LOG_DIR = REPO_ROOT / "runtime_logs"
REMOTE_ROOT = "/home/user/sdk_deploy_tty"
REMOTE_LOG_DIR = f"{REMOTE_ROOT}/runtime_logs"


def now_tag():
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def run(cmd, *, check=True, timeout=None):
    print("+ " + " ".join(cmd), flush=True)
    return subprocess.run(cmd, check=check, timeout=timeout, text=True)


def capture(cmd, *, timeout=None):
    print("+ " + " ".join(cmd), flush=True)
    return subprocess.run(cmd, check=False, timeout=timeout, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def ssh_cmd(host, remote_cmd, *, jump=None, timeout=None, check=True):
    cmd = ["ssh", "-o", "ConnectTimeout=8"]
    if jump:
        cmd += ["-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/tmp/known_hosts_106_via_103", "-J", jump]
    cmd += [host, remote_cmd]
    return run(cmd, check=check, timeout=timeout)


def ssh_capture(host, remote_cmd, *, jump=None, timeout=None):
    cmd = ["ssh", "-o", "ConnectTimeout=8"]
    if jump:
        cmd += ["-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/tmp/known_hosts_106_via_103", "-J", jump]
    cmd += [host, remote_cmd]
    return capture(cmd, timeout=timeout)


def sudo_script(host, script, *, jump=None, timeout=None, check=True):
    encoded = base64.b64encode(script.encode("utf-8")).decode("ascii")
    runner = f"echo {encoded} | base64 -d | bash"
    ssh = ["ssh", "-o", "ConnectTimeout=8"]
    if jump:
        ssh += ["-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/tmp/known_hosts_106_via_103", "-J", jump]
    password = getattr(sudo_script, "password", "'")
    ssh += [host, f"printf {shell_quote(password + chr(10))} | sudo -S -p '' bash -lc {shell_quote(runner)}"]
    print("+ " + " ".join(ssh), flush=True)
    return subprocess.run(ssh, text=True, check=check, timeout=timeout)


def shell_quote(text):
    return "'" + str(text).replace("'", "'\"'\"'") + "'"


def start_local_background(args, name, cmd, log_file):
    log_file.parent.mkdir(parents=True, exist_ok=True)
    if args.dry_run:
        print("[dry-run] " + " ".join(cmd) + f" > {log_file} 2>&1 &")
        return None
    with log_file.open("ab", buffering=0) as f:
        proc = subprocess.Popen(
            cmd,
            cwd=str(REPO_ROOT),
            stdout=f,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            env={**os.environ, "PYTHONUNBUFFERED": "1"},
        )
    print(f"[local] started {name}: pid={proc.pid} log={log_file}", flush=True)
    return proc.pid


def start_103(args, tag):
    script = f"""
set -e
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh
mkdir -p {REMOTE_LOG_DIR}/{tag}
systemctl restart lio_perception.service height_map_nav.service || true
/opt/robot/share/lio_perception/scripts/start.sh || true
ros2 service call /HEIGHT_MAP_ENABLE drdds/srv/StdSrvInt32 "{{command: 1}}" || true
ros2 service call /SDK_MODE drdds/srv/StdSrvInt32 "{{command: {args.sdk_rate}}}"
cd {REMOTE_ROOT}
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh
source install/setup.bash
log={REMOTE_LOG_DIR}/{tag}/rl_deploy_103.log
pkill -TERM -f {REMOTE_ROOT}/install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy || true
nohup ros2 run m20_sdk_deploy rl_deploy > "$log" 2>&1 < /dev/null &
echo "rl_deploy log=$log"
sleep 1
pgrep -af "{REMOTE_ROOT}/install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy|ros2 run m20_sdk_deploy rl_deploy" || true
"""
    if args.dry_run:
        print("[dry-run] 103 script:\n" + script)
        return
    sudo_script(args.host103, script, timeout=60)


def start_106(args, tag):
    script = f"""
set -e
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh 2>/dev/null || true
cd {REMOTE_ROOT}
mkdir -p {REMOTE_LOG_DIR}/{tag}
log={REMOTE_LOG_DIR}/{tag}/noisy_elevation_106.log
pkill -TERM -f noisy_elevation_node.py || true
nohup python3 src/M20_sdk_deploy/scripts/noisy_elevation_node.py \\
  --ros-args \\
  -p lidar_topic:=/CLOUD_REGISTERED_BODY \\
  -p height_topic:=/height_map \\
  -p terrain_cache_scan:=true \\
  -p fk_height_scan:=false \\
  -p zero_height_scan:=false \\
  -p cache_ttl_sec:={args.cache_ttl_sec} \\
  -p cache_radius_m:={args.cache_radius_m} \\
  > "$log" 2>&1 < /dev/null &
echo "noisy_elevation log=$log"
sleep 1
pgrep -af noisy_elevation_node.py || true
"""
    if args.dry_run:
        print("[dry-run] 106 script:\n" + script)
        return
    sudo_script(args.host106, script, jump=args.jump, timeout=60)


def start_local(args, tag):
    local_dir = LOCAL_LOG_DIR / tag
    joystick_cmd = [
        sys.executable,
        "-u",
        str(REPO_ROOT / "src/M20_sdk_deploy/scripts/joy_tcp_sender.py"),
        "--ip",
        args.robot_ip,
        "--port",
        str(args.joy_port),
        "--device",
        args.joy_device,
        "--rate",
        str(args.joy_rate),
    ]
    start_local_background(args, "joystick_sender", joystick_cmd, local_dir / "joystick_sender.log")

    if args.web_visualizer:
        web_cmd = [
            sys.executable,
            "-u",
            str(REPO_ROOT / "src/M20_sdk_deploy/scripts/visualize_height_map_web3d_ssh.py"),
            "--remote",
            args.host106,
            "--jump",
            args.jump,
            "--rate",
            str(args.visual_rate),
            "--stride",
            str(args.visual_stride),
            "--port",
            str(args.visual_port),
        ]
        if args.no_open_browser:
            web_cmd.append("--no-open")
        start_local_background(args, "height_map_web3d", web_cmd, local_dir / "height_map_web3d.log")
        print(f"[local] height-map web UI: http://127.0.0.1:{args.visual_port}/", flush=True)


def status(args):
    print("== local ==")
    local = capture(["pgrep", "-af", "[j]oy_tcp_sender.py|[v]isualize_height_map_web3d_ssh.py"])
    print(local.stdout or "(none)")
    print("== 103 ==")
    out103 = ssh_capture(args.host103, "pgrep -af '[s]dk_deploy_tty|[m]20_sdk_deploy|[r]l_deploy|[l]io_ddsnode|[r]slidar' || true", timeout=10)
    print(out103.stdout)
    print("== 106 ==")
    out106 = ssh_capture(args.host106, "pgrep -af '[n]oisy_elevation_node.py|[h]andler|[r]slidar' || true", jump=args.jump, timeout=10)
    print(out106.stdout)


def stop(args):
    print("== stopping local joystick/web visualizer ==")
    local = subprocess.run(
        ["pgrep", "-f", "[j]oy_tcp_sender.py|[v]isualize_height_map_web3d_ssh.py"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    for pid_s in local.stdout.split():
        pid = int(pid_s)
        if pid != os.getpid():
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    time.sleep(1)

    print("== stopping 106 noisy elevation ==")
    sudo_script(args.host106, "pkill -TERM -f noisy_elevation_node.py || true\n", jump=args.jump, timeout=15, check=False)

    print("== stopping 103 deploy rl and exiting SDK mode ==")
    script = f"""
set +e
pkill -TERM -f {REMOTE_ROOT}/install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy
pkill -TERM -f 'ros2 run m20_sdk_deploy rl_deploy'
sleep 1
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh 2>/dev/null || true
ros2 service call /SDK_MODE drdds/srv/StdSrvInt32 "{{command: 0}}"
"""
    sudo_script(args.host103, script, timeout=30, check=False)
    status(args)


def start(args):
    tag = args.tag or now_tag()
    print(f"[deploy] tag={tag}")
    print("[1/4] 103: LIO/height map, SDK mode, rl_deploy")
    start_103(args, tag)
    print("[2/4] 106: noisy_elevation_node")
    start_106(args, tag)
    print("[3/4] local: joystick sender")
    print("[4/4] local: web height-map visualizer" if args.web_visualizer else "[4/4] local: web visualizer disabled")
    start_local(args, tag)
    print("[deploy] startup commands completed")
    if args.dry_run:
        print("[dry-run] skip status checks")
    else:
        status(args)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["start", "stop", "status"])
    parser.add_argument("--host103", default="user@10.21.41.1")
    parser.add_argument("--host106", default="user@10.21.33.106")
    parser.add_argument("--jump", default="user@10.21.41.1")
    parser.add_argument("--sudo-password", default=os.environ.get("M20_SUDO_PASSWORD", "'"))
    parser.add_argument("--robot-ip", default="10.21.41.1")
    parser.add_argument("--sdk-rate", type=int, default=200)
    parser.add_argument("--joy-port", type=int, default=9999)
    parser.add_argument("--joy-device", default="/dev/input/js0")
    parser.add_argument("--joy-rate", type=float, default=200.0)
    parser.add_argument("--cache-ttl-sec", type=float, default=8.0)
    parser.add_argument("--cache-radius-m", type=float, default=3.0)
    parser.add_argument("--web-visualizer", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--visual-rate", type=float, default=3.0)
    parser.add_argument("--visual-stride", type=int, default=2)
    parser.add_argument("--visual-port", type=int, default=8765)
    parser.add_argument("--no-open-browser", action="store_true")
    parser.add_argument("--tag", default="")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    sudo_script.password = args.sudo_password
    if args.command == "start":
        start(args)
    elif args.command == "stop":
        stop(args)
    else:
        status(args)


if __name__ == "__main__":
    main()
