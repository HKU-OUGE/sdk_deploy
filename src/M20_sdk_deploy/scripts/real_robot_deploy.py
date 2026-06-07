#!/usr/bin/env python3
"""Start/stop the M20 real-robot deployment from the local computer.

Default network topology:
  local PC -> 103 WiFi host: user@10.21.41.1
  optional legacy perception: local PC -> 106 through 103: user@10.21.33.106

The script starts only this repo's deployment processes.  It leaves vendor
processes under /opt/robot running, except for restarting LIO/height-map
services when requested by the start sequence.
"""

import argparse
import base64
import datetime as _dt
import getpass
import os
import signal
import shutil
import socket
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
    password = getattr(sudo_script, "password", "'")
    askpass_script = (
        "askpass=$(mktemp) && "
        "trap 'rm -f \"$askpass\"' EXIT && "
        "cat > \"$askpass\" <<'ASKPASS'\n"
        "#!/bin/sh\n"
        f"printf %s {shell_quote(password)}\n"
        "ASKPASS\n"
        "chmod 700 \"$askpass\" && "
        f"SUDO_ASKPASS=\"$askpass\" sudo -A -p '' bash -lc {shell_quote(runner)}"
    )
    ssh = ["ssh", "-o", "ConnectTimeout=8"]
    if jump:
        ssh += ["-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/tmp/known_hosts_106_via_103", "-J", jump]
    ssh += [host, askpass_script]
    line_count = len([line for line in script.splitlines() if line.strip()])
    print("+ " + " ".join(ssh[:-1]) + f" <sudo remote script: {line_count} lines>", flush=True)
    return subprocess.run(ssh, text=True, check=check, timeout=timeout)


def shell_quote(text):
    return "'" + str(text).replace("'", "'\"'\"'") + "'"


def require_file(path, label):
    if not Path(path).exists():
        raise FileNotFoundError(f"{label} not found: {path}")


def local_port_is_free(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.2)
        return sock.connect_ex(("127.0.0.1", int(port))) != 0


def preflight_local(args):
    print("== preflight: local ==")
    require_file(REPO_ROOT / "src/M20_sdk_deploy/scripts/joy_tcp_sender.py", "joystick sender")
    require_file(REPO_ROOT / "src/M20_sdk_deploy/scripts/visualize_height_map_web3d_ssh.py", "height-map visualizer")
    require_file(REPO_ROOT / "src/M20_sdk_deploy/scripts/noisy_elevation_node.py", "noisy elevation node")
    if not Path(args.joy_device).exists():
        print(f"[warn] joystick device does not exist yet: {args.joy_device}", flush=True)
    if args.web_visualizer and not local_port_is_free(args.visual_port):
        raise RuntimeError(f"local visualizer port is already in use: 127.0.0.1:{args.visual_port}")
    if shutil.which("ssh") is None:
        raise RuntimeError("ssh command not found")
    if args.sync_remote_scripts and shutil.which("scp") is None:
        raise RuntimeError("scp command not found")


def cleanup_local_deploy_processes(args):
    if args.dry_run:
        return
    for unit in (
        f"m20-official-heightmap-{args.visual_port}.service",
        f"m20-heightpoints-{args.visual_port}.service",
        "m20-heightmap-8767.service",
    ):
        subprocess.run(
            ["systemctl", "--user", "stop", unit],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        subprocess.run(
            ["systemctl", "--user", "reset-failed", unit],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
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
    time.sleep(0.5)


def scp_file(local_path, host, remote_path, *, jump=None, timeout=20):
    cmd = ["scp", "-o", "ConnectTimeout=8"]
    if jump:
        cmd += ["-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/tmp/known_hosts_106_via_103", "-J", jump]
    cmd += [str(local_path), f"{host}:{remote_path}"]
    return run(cmd, timeout=timeout)


def sync_remote_scripts(args):
    if not args.sync_remote_scripts:
        return
    local_noisy = REPO_ROOT / "src/M20_sdk_deploy/scripts/noisy_elevation_node.py"
    remote_dir = f"{REMOTE_ROOT}/src/M20_sdk_deploy/scripts"
    remote_noisy = f"{remote_dir}/noisy_elevation_node.py"
    if args.dry_run:
        print(f"[dry-run] sync {local_noisy} -> {args.host103}:{remote_noisy}")
        if args.legacy_106_perception:
            print(f"[dry-run] sync {local_noisy} -> {args.host106}:{remote_noisy} via {args.jump}")
        return
    print("== sync remote python scripts ==", flush=True)
    ssh_cmd(args.host103, f"mkdir -p {remote_dir}", timeout=15)
    scp_file(local_noisy, args.host103, remote_noisy, timeout=20)
    if args.legacy_106_perception:
        ssh_cmd(args.host106, f"mkdir -p {remote_dir}", jump=args.jump, timeout=15)
        scp_file(local_noisy, args.host106, remote_noisy, jump=args.jump, timeout=20)


def preflight_remote(args):
    print("== preflight: 103 ==")
    script103 = f"""
set -e
test -d {REMOTE_ROOT}
test -f {REMOTE_ROOT}/install/setup.bash
test -x {REMOTE_ROOT}/install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy
test -x {REMOTE_ROOT}/install/m20_sdk_deploy/lib/m20_sdk_deploy/lidar_to_scan_cpp
test -f {REMOTE_ROOT}/src/M20_sdk_deploy/scripts/noisy_elevation_node.py
test -f /opt/ros/foxy/setup.bash
test -f /opt/robot/scripts/setup_ros2.sh
command -v ros2 >/dev/null || true
systemctl is-active lio_perception.service >/dev/null || true
systemctl is-active height_map_nav.service >/dev/null || true
python3 -m py_compile {REMOTE_ROOT}/src/M20_sdk_deploy/scripts/noisy_elevation_node.py
echo "103 preflight ok"
"""
    sudo_script(args.host103, script103, timeout=25)

    if not args.legacy_106_perception:
        return

    print("== preflight: 106 ==")
    script106 = f"""
set -e
test -d {REMOTE_ROOT}
test -f {REMOTE_ROOT}/src/M20_sdk_deploy/scripts/noisy_elevation_node.py
test -f /opt/ros/foxy/setup.bash
echo "106 preflight ok"
"""
    sudo_script(args.host106, script106, jump=args.jump, timeout=25)


def preflight(args):
    if args.dry_run:
        print("[dry-run] skip preflight checks")
        return
    preflight_local(args)
    sync_remote_scripts(args)
    preflight_remote(args)


def start_local_background(args, name, cmd, log_file, *, env_extra=None):
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
            env={**os.environ, "PYTHONUNBUFFERED": "1", **(env_extra or {})},
        )
    print(f"[local] started {name}: pid={proc.pid} log={log_file}", flush=True)
    return proc.pid


def start_103(args, tag):
    script = f"""
set -e
run_with_timeout() {{
  seconds="$1"
  shift
  "$@" &
  cmd_pid=$!
  deadline=$(awk "BEGIN {{ printf \\"%.0f\\", $seconds * 10 }}")
  elapsed=0
  while kill -0 "$cmd_pid" 2>/dev/null; do
    if [ "$elapsed" -ge "$deadline" ]; then
      kill -TERM "$cmd_pid" 2>/dev/null || true
      sleep 0.2
      kill -KILL "$cmd_pid" 2>/dev/null || true
      wait "$cmd_pid" 2>/dev/null || true
      return 124
    fi
    sleep 0.1
    elapsed=$((elapsed + 1))
  done
  wait "$cmd_pid"
}}
drdds_int32_call_python() {{
  service_name="$1"
  command_value="$2"
  timeout_sec="$3"
  node_name="$4"
  tmp=$(mktemp)
  set +e
  run_with_timeout "$timeout_sec" python3 - "$service_name" "$command_value" "$timeout_sec" "$node_name" <<'PY' > "$tmp" 2>&1
import os
import sys
import time

import rclpy
from rclpy.node import Node
from drdds.srv import StdSrvInt32


service_name = sys.argv[1]
command_value = int(sys.argv[2])
timeout_sec = float(sys.argv[3])
node_name = sys.argv[4]

try:
    rclpy.init(args=None)
    node = Node(node_name)
    client = node.create_client(StdSrvInt32, service_name)
    deadline = time.time() + timeout_sec
    while not client.wait_for_service(timeout_sec=0.2):
        if time.time() >= deadline:
            print(f"{{service_name}} unavailable", flush=True)
            os._exit(2)
    req = StdSrvInt32.Request()
    req.command = command_value
    future = client.call_async(req)
    while not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        print(f"{{service_name}} timeout", flush=True)
        os._exit(3)
    result = int(getattr(future.result(), "result", 0))
    print(f"{{service_name}} result={{result}}", flush=True)
    os._exit(0 if result == 1 else 4)
except Exception as exc:
    print(f"{{service_name}} exception={{type(exc).__name__}}: {{exc}}", flush=True)
    os._exit(5)
PY
  rc=$?
  set -e
  cat "$tmp"
  if grep -q "$service_name result=1" "$tmp" || grep -q "$service_name result: 1" "$tmp"; then
    rm -f "$tmp"
    return 0
  fi
  rm -f "$tmp"
  return 1
}}
ros2_int32_call_fallback() {{
  service_name="$1"
  command_value="$2"
  timeout_sec="$3"
  tmp=$(mktemp)
  set +e
  run_with_timeout "$timeout_sec" ros2 service call "$service_name" drdds/srv/StdSrvInt32 "{{command: $command_value}}" > "$tmp" 2>&1
  rc=$?
  set -e
  cat "$tmp"
  if grep -q 'result=1' "$tmp" || grep -q 'result: 1' "$tmp"; then
    rm -f "$tmp"
    return 0
  fi
  rm -f "$tmp"
  return 1
}}
drdds_int32_call() {{
  service_name="$1"
  command_value="$2"
  timeout_sec="$3"
  node_name="$4"
  if drdds_int32_call_python "$service_name" "$command_value" "$timeout_sec" "$node_name"; then
    return 0
  fi
  echo "[warn] python service call failed for $service_name; falling back to ros2 CLI"
  ros2_int32_call_fallback "$service_name" "$command_value" "$timeout_sec"
}}
height_map_enable_call() {{
  drdds_int32_call /HEIGHT_MAP_ENABLE 1 {args.height_enable_timeout_sec} height_map_enable_client
}}
sdk_mode_call() {{
  attempt=1
  while [ "$attempt" -le {args.sdk_mode_attempts} ]; do
    echo "[103] SDK_MODE attempt $attempt/{args.sdk_mode_attempts}"
    if drdds_int32_call /SDK_MODE {args.sdk_rate} {args.sdk_mode_timeout_sec} "sdk_mode_client_$attempt"; then
      return 0
    fi
    sleep 1
    attempt=$((attempt + 1))
  done
  return 1
}}
sdk_mode_call_cli_legacy() {{
  tmp=$(mktemp)
  set +e
  run_with_timeout {args.sdk_mode_timeout_sec} ros2 service call /SDK_MODE drdds/srv/StdSrvInt32 "{{command: {args.sdk_rate}}}" > "$tmp" 2>&1
  rc=$?
  set -e
  cat "$tmp"
  if grep -q 'result=1' "$tmp" || grep -q 'result: 1' "$tmp"; then
    rm -f "$tmp"
    return 0
  fi
  rm -f "$tmp"
  return 1
}}
cloud_frame_available() {{
  tmp=$(mktemp)
  set +e
  run_with_timeout 6.0 python3 - <<'PY' > "$tmp" 2>&1
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class Probe(Node):
    def __init__(self):
        super().__init__("cloud_registered_probe")
        self.ok = False
        self.create_subscription(PointCloud2, "/CLOUD_REGISTERED_BODY", self.cb, 10)

    def cb(self, msg):
        if msg.width * msg.height > 0:
            self.ok = True


rclpy.init()
node = Probe()
deadline = time.time() + 4.0
while time.time() < deadline and not node.ok:
    rclpy.spin_once(node, timeout_sec=0.2)
ok = node.ok
print("CLOUD_FRAME ok" if ok else "CLOUD_FRAME missing", flush=True)
os._exit(0 if ok else 2)
PY
  rc=$?
  set -e
  cat "$tmp"
  if grep -q 'CLOUD_FRAME ok' "$tmp"; then
    rm -f "$tmp"
    return 0
  fi
  rm -f "$tmp"
  return 1
}}
height_map_frame_available() {{
  tmp=$(mktemp)
  set +e
  run_with_timeout 6.0 python3 - <<'PY' > "$tmp" 2>&1
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class Probe(Node):
    def __init__(self):
        super().__init__("height_map_probe")
        self.frame = None
        self.create_subscription(PointCloud2, "/height_map", self.cb, 10)

    def cb(self, msg):
        if msg.width * msg.height > 0:
            self.frame = (int(msg.width), int(msg.height))


rclpy.init(args=None)
node = Probe()
deadline = time.time() + 4.0
while time.time() < deadline and node.frame is None:
    rclpy.spin_once(node, timeout_sec=0.2)
if node.frame is None:
    print("HEIGHT_MAP_FRAME missing", flush=True)
    os._exit(2)
width, height = node.frame
print(f"HEIGHT_MAP_FRAME ok {{width}}x{{height}}", flush=True)
os._exit(0)
PY
  rc=$?
  set -e
  cat "$tmp"
  if grep -q 'HEIGHT_MAP_FRAME ok' "$tmp"; then
    rm -f "$tmp"
    return 0
  fi
  rm -f "$tmp"
  return 1
}}
scan_frame_available() {{
  tmp=$(mktemp)
  set +e
  run_with_timeout 8.0 python3 - <<'PY' > "$tmp" 2>&1
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class Probe(Node):
    def __init__(self):
        super().__init__("legacy_scan_probe")
        self.frame = None
        self.create_subscription(Float32MultiArray, "/scan/multi_layer_features_array", self.cb, 10)

    def cb(self, msg):
        data = list(msg.data)
        if len(data) == 992:
            finite = [v for v in data if math.isfinite(v)]
            if finite:
                self.frame = (len(data), min(finite), max(finite))


rclpy.init()
node = Probe()
deadline = time.time() + 6.0
while time.time() < deadline and node.frame is None:
    rclpy.spin_once(node, timeout_sec=0.2)
if node.frame is None:
    print("SCAN_FRAME missing", flush=True)
    os._exit(2)
dim, v_min, v_max = node.frame
print(f"SCAN_FRAME ok dim={{dim}} min={{v_min:.3f}} max={{v_max:.3f}}", flush=True)
os._exit(0)
PY
  rc=$?
  set -e
  cat "$tmp"
  if grep -q 'SCAN_FRAME ok' "$tmp"; then
    rm -f "$tmp"
    return 0
  fi
  rm -f "$tmp"
  return 1
}}
start_lio_with_retries() {{
  attempt=1
  ok=1
  while [ "$attempt" -le {args.lio_start_attempts} ]; do
    echo "[103] starting LIO attempt $attempt/{args.lio_start_attempts}"
    tmp=$(mktemp)
    set +e
    run_with_timeout {args.lio_start_timeout_sec} /opt/robot/share/lio_perception/scripts/start.sh > "$tmp" 2>&1
    rc=$?
    set -e
    cat "$tmp"
    if grep -q '调用成功' "$tmp" || grep -q 'res: 1' "$tmp"; then
      echo "[103] LIO start.sh returned success on attempt $attempt"
      ok=0
      if cloud_frame_available; then
        rm -f "$tmp"
        return 0
      fi
      echo "[warn] LIO command returned success but /CLOUD_REGISTERED_BODY has no frame yet"
    else
      echo "[warn] LIO start.sh did not report success on attempt $attempt (rc=$rc)"
    fi
    rm -f "$tmp"
    sleep 2
    attempt=$((attempt + 1))
  done
  if [ "$ok" -ne 0 ]; then
    echo "[warn] LIO start.sh did not return success after {args.lio_start_attempts} attempt(s); continuing"
  fi
  return "$ok"
}}
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh
mkdir -p {REMOTE_LOG_DIR}/{tag}
if systemctl is-active --quiet lio_perception.service && systemctl is-active --quiet height_map_nav.service && cloud_frame_available && height_map_frame_available; then
  echo "[103] LIO and official /height_map already have frames; skip service restart"
else
  systemctl restart lio_perception.service height_map_nav.service || true
  start_lio_with_retries || true
fi
if height_map_frame_available; then
  echo "[103] /height_map already has frames; skip HEIGHT_MAP_ENABLE"
else
  height_map_enable_call || echo "[warn] HEIGHT_MAP_ENABLE timed out or failed; continuing"
  height_map_frame_available || echo "[warn] /height_map still has no frame after HEIGHT_MAP_ENABLE"
fi
if ! sdk_mode_call; then
  echo "[error] SDK_MODE command failed or timed out"
  exit 1
fi
cd {REMOTE_ROOT}
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh
source install/setup.bash
if [ "{str(args.legacy_scan).lower()}" = "true" ]; then
  scan_log={REMOTE_LOG_DIR}/{tag}/lidar_to_scan_103.log
  pkill -TERM -f {REMOTE_ROOT}/install/m20_sdk_deploy/lib/m20_sdk_deploy/lidar_to_scan_cpp || true
  pkill -TERM -f 'ros2 run m20_sdk_deploy lidar_to_scan_cpp' || true
  nohup ros2 run m20_sdk_deploy lidar_to_scan_cpp --ros-args -p lidar_topic:={args.scan_lidar_topic} > "$scan_log" 2>&1 < /dev/null &
  echo "lidar_to_scan log=$scan_log"
  sleep 1
  pgrep -af "{REMOTE_ROOT}/install/m20_sdk_deploy/lib/m20_sdk_deploy/lidar_to_scan_cpp|ros2 run m20_sdk_deploy lidar_to_scan_cpp" || true
  if ! scan_frame_available; then
    echo "[warn] /scan/multi_layer_features_array is missing; platform/crawl policies will see no-hit scan values"
  fi
fi
if [ "{str(not args.legacy_106_perception).lower()}" = "true" ]; then
  perception_log={REMOTE_LOG_DIR}/{tag}/noisy_elevation_103_official_heightmap.log
  pkill -TERM -f {REMOTE_ROOT}/src/M20_sdk_deploy/scripts/noisy_elevation_node.py || true
  pkill -TERM -f 'src/M20_sdk_deploy/scripts/noisy_elevation_node.py' || true
  nohup python3 -u src/M20_sdk_deploy/scripts/noisy_elevation_node.py \\
    --ros-args \\
    -p lidar_topic:=/CLOUD_REGISTERED_BODY \\
    -p height_topic:=/height_map \\
    -p height_map_mode:={args.official_height_map_mode} \\
    -p height_invalid_mode:={args.height_invalid_mode} \\
    -p terrain_cache_scan:=false \\
    -p fk_height_scan:=false \\
    -p zero_height_scan:=false \\
    -p scan_cache_ttl_sec:={args.scan_cache_ttl_sec} \\
    -p scan_cache_min_valid_bins:={args.scan_cache_min_valid_bins} \\
    > "$perception_log" 2>&1 < /dev/null &
  echo "official height-map noisy_elevation log=$perception_log"
  sleep 1
  pgrep -af "{REMOTE_ROOT}/src/M20_sdk_deploy/scripts/noisy_elevation_node.py|src/M20_sdk_deploy/scripts/noisy_elevation_node.py" || true
fi
log={REMOTE_LOG_DIR}/{tag}/rl_deploy_103.log
pkill -TERM -f {REMOTE_ROOT}/install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy || true
nohup ros2 run m20_sdk_deploy rl_deploy > "$log" 2>&1 < /dev/null &
echo "rl_deploy log=$log"
sleep 1
pgrep -af "{REMOTE_ROOT}/install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy|ros2 run m20_sdk_deploy rl_deploy" || true
ss -ltnp 2>/dev/null | grep ':9999' || true
"""
    if args.dry_run:
        print("[dry-run] 103 script:\n" + script)
        return
    sudo_script(args.host103, script, timeout=args.start_103_timeout_sec)


def wait_for_103_perception_frame(args):
    if args.dry_run:
        print("[dry-run] skip waiting for 103 noisy_elevation frame")
        return
    check_script = r"""
set -e
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh
python3 - <<'PY'
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray


class Probe(Node):
    def __init__(self):
        super().__init__("official_noisy_elevation_probe")
        self.frame = None
        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Float32MultiArray, "/perception/noisy_elevation_array", self.cb, qos)

    def cb(self, msg):
        data = np.asarray(msg.data, dtype=np.float32)
        if data.size == 691 and np.all(np.isfinite(data)):
            h = data[:187]
            self.frame = (data.size, float(h.min()), float(h.max()), float(h.mean()))


rclpy.init()
node = Probe()
deadline = time.time() + 8.0
while time.time() < deadline and node.frame is None:
    rclpy.spin_once(node, timeout_sec=0.2)
if node.frame is None:
    print("NOISY_ELEVATION_FRAME missing", flush=True)
    os._exit(2)
dim, h_min, h_max, h_mean = node.frame
print(f"NOISY_ELEVATION_FRAME ok dim={dim} height_min={h_min:.3f} height_max={h_max:.3f} height_mean={h_mean:.3f}", flush=True)
os._exit(0)
PY
"""
    out = sudo_script(args.host103, check_script, timeout=20, check=False)
    if out.returncode != 0:
        raise RuntimeError("103 /perception/noisy_elevation_array did not produce a valid 691-dim frame")


def ensure_103_lio_data(args):
    script = f"""
set -e
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh
for i in $(seq 1 {args.lio_recover_attempts}); do
  echo "[103] extra LIO enable attempt $i/{args.lio_recover_attempts}"
  set +e
  /opt/robot/share/lio_perception/scripts/start.sh
  rc=$?
  set -e
  echo "[103] extra LIO enable rc=$rc"
  sleep 2
done
"""
    sudo_script(args.host103, script, timeout=max(30, args.lio_recover_attempts * 18), check=False)


def wait_for_106_height_map_frame(args):
    if args.dry_run:
        print("[dry-run] skip waiting for 106 height-map frame")
        return
    check_script = r"""
set -e
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh 2>/dev/null || true
python3 - <<'PY'
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Probe(Node):
    def __init__(self):
        super().__init__("height_map_probe")
        self.frame = None
        self.create_subscription(PointCloud2, "/height_map", self.cb, 10)

    def cb(self, msg):
        pts = list(point_cloud2.read_points(msg, field_names=("z",), skip_nans=False))
        if not pts:
            return
        arr = np.asarray(pts)
        if arr.dtype.names is not None:
            z = arr["z"].astype(np.float32)
        else:
            z = arr.astype(np.float32).reshape(-1)
        finite = np.isfinite(z)
        if finite.any():
            self.frame = (int(finite.sum()), float(np.nanmin(z)), float(np.nanmax(z)))


rclpy.init()
node = Probe()
deadline = time.time() + 8.0
while time.time() < deadline and node.frame is None:
    rclpy.spin_once(node, timeout_sec=0.2)
if node.frame is None:
    print("HEIGHT_MAP_FRAME missing")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(2)
finite, z_min, z_max = node.frame
print(f"HEIGHT_MAP_FRAME ok finite={finite} z_min={z_min:.3f} z_max={z_max:.3f}")
node.destroy_node()
rclpy.shutdown()
PY
"""
    for attempt in range(1, args.height_frame_attempts + 1):
        print(f"[106] wait for /height_map frame attempt {attempt}/{args.height_frame_attempts}", flush=True)
        out = sudo_script(args.host106, check_script, jump=args.jump, timeout=20, check=False)
        if out.returncode == 0:
            print("[106] /height_map frame is available", flush=True)
            return
        if attempt < args.height_frame_attempts:
            print("[warn] 106 did not receive /height_map; re-enabling LIO on 103", flush=True)
            ensure_103_lio_data(args)
    raise RuntimeError("106 did not receive /height_map frames; web visualizer would be blank")


def wait_for_103_control_port(args):
    if args.dry_run:
        print("[dry-run] skip waiting for 103 TCP control port")
        return
    deadline = time.time() + args.wait_control_sec
    while time.time() < deadline:
        out = ssh_capture(args.host103, "ss -ltn 2>/dev/null | grep -q ':9999' && echo ready || true", timeout=10)
        if "ready" in out.stdout:
            print("[103] TCP control port 9999 is listening", flush=True)
            return
        time.sleep(1.0)
    raise RuntimeError(f"103 rl_deploy did not open TCP :9999 within {args.wait_control_sec:.0f}s")


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
  -p height_invalid_mode:={args.height_invalid_mode} \\
  -p terrain_cache_scan:=true \\
  -p fk_height_scan:=false \\
  -p zero_height_scan:=false \\
  -p cache_ttl_sec:={args.cache_ttl_sec} \\
  -p cache_radius_m:={args.cache_radius_m} \\
  -p scan_cache_ttl_sec:={args.scan_cache_ttl_sec} \\
  -p scan_cache_min_valid_bins:={args.scan_cache_min_valid_bins} \\
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
            args.visual_remote or args.host103,
            "--jump",
            args.visual_jump,
            "--height-topic",
            args.visual_height_topic,
            "--rate",
            str(args.visual_rate),
            "--stride",
            str(args.visual_stride),
            "--port",
            str(args.visual_port),
        ]
        if args.no_open_browser:
            web_cmd.append("--no-open")
        start_local_background(
            args,
            "height_map_web3d",
            web_cmd,
            local_dir / "height_map_web3d.log",
            env_extra={"M20_SUDO_PASSWORD": args.sudo_password},
        )
        print(f"[local] height-map web UI: http://127.0.0.1:{args.visual_port}/", flush=True)


def status(args):
    print("== local ==")
    local = capture(["pgrep", "-af", "[j]oy_tcp_sender.py|[v]isualize_height_map_web3d_ssh.py"])
    print(local.stdout or "(none)")
    sockets = capture(["bash", "-lc", f"ss -tnlp 2>/dev/null | grep -E ':{args.visual_port}|:{args.joy_port}' || true"])
    print(sockets.stdout or "(no local matching sockets)")
    print("== 103 ==")
    out103 = ssh_capture(
        args.host103,
        "pgrep -af '[s]dk_deploy_tty|[m]20_sdk_deploy|[r]l_deploy|[l]idar_to_scan|[n]oisy_elevation_node|[h]eight_map_nav|[l]io_ddsnode|[r]slidar' || true; "
        "ss -ltnp 2>/dev/null | grep ':9999' || true",
        timeout=10,
    )
    print(out103.stdout)
    if args.legacy_106_perception:
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
    local_left = subprocess.run(
        ["pgrep", "-f", "[j]oy_tcp_sender.py|[v]isualize_height_map_web3d_ssh.py"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    for pid_s in local_left.stdout.split():
        pid = int(pid_s)
        if pid != os.getpid():
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    if args.legacy_106_perception:
        print("== stopping 106 noisy elevation ==")
        sudo_script(args.host106, "pkill -TERM -f noisy_elevation_node.py || true\n", jump=args.jump, timeout=15, check=False)

    print("== stopping 103 deploy rl and exiting SDK mode ==")
    script = f"""
set +e
pkill -TERM -f {REMOTE_ROOT}/src/M20_sdk_deploy/scripts/noisy_elevation_node.py
pkill -TERM -f 'src/M20_sdk_deploy/scripts/noisy_elevation_node.py'
pkill -TERM -f {REMOTE_ROOT}/install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy
pkill -TERM -f 'ros2 run m20_sdk_deploy rl_deploy'
pkill -TERM -f {REMOTE_ROOT}/install/m20_sdk_deploy/lib/m20_sdk_deploy/lidar_to_scan_cpp
pkill -TERM -f 'ros2 run m20_sdk_deploy lidar_to_scan_cpp'
sleep 1
source /opt/ros/foxy/setup.bash
source /opt/robot/scripts/setup_ros2.sh 2>/dev/null || true
python3 - <<'PY' || ros2 service call /SDK_MODE drdds/srv/StdSrvInt32 "{{command: 0}}"
import os
import time

import rclpy
from rclpy.node import Node
from drdds.srv import StdSrvInt32


try:
    rclpy.init(args=None)
    node = Node("sdk_mode_exit_client")
    client = node.create_client(StdSrvInt32, "/SDK_MODE")
    deadline = time.time() + 10.0
    while not client.wait_for_service(timeout_sec=0.2):
        if time.time() >= deadline:
            print("/SDK_MODE exit unavailable", flush=True)
            os._exit(2)
    req = StdSrvInt32.Request()
    req.command = 0
    future = client.call_async(req)
    while not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        print("/SDK_MODE exit timeout", flush=True)
        os._exit(3)
    result = int(getattr(future.result(), "result", 0))
    print("/SDK_MODE exit result=%d" % result, flush=True)
    os._exit(0 if result == 1 else 4)
except Exception as exc:
    print("/SDK_MODE exit exception=%s: %s" % (type(exc).__name__, exc), flush=True)
    os._exit(5)
PY
"""
    sudo_script(args.host103, script, timeout=30, check=False)
    status(args)


def start(args):
    tag = args.tag or now_tag()
    print(f"[deploy] tag={tag}")
    cleanup_local_deploy_processes(args)
    if not args.skip_preflight:
        preflight(args)
    total_steps = 5 if args.web_visualizer else 4
    if args.legacy_106_perception:
        total_steps += 1
    print(f"[1/{total_steps}] 103: LIO/official height map, SDK mode, rl_deploy")
    start_103(args, tag)
    next_step = 2
    if args.legacy_106_perception:
        print(f"[{next_step}/{total_steps}] 106: legacy noisy_elevation_node")
        start_106(args, tag)
        next_step += 1
        if args.web_visualizer:
            print(f"[{next_step}/{total_steps}] wait for 106 height-map frame")
            wait_for_106_height_map_frame(args)
            next_step += 1
    else:
        print(f"[{next_step}/{total_steps}] wait for 103 official noisy_elevation frame")
        wait_for_103_perception_frame(args)
        next_step += 1
    control_step = next_step
    print(f"[{control_step}/{total_steps}] wait for 103 TCP control port")
    wait_for_103_control_port(args)
    print(f"[{control_step + 1}/{total_steps}] local: joystick sender")
    print(f"[{control_step + 2}/{total_steps}] local: web height-map visualizer" if args.web_visualizer else f"[{control_step + 2}/{total_steps}] local: web visualizer disabled")
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
    parser.add_argument("--ask-sudo-password", action="store_true", help="prompt once locally and reuse for 103/106 sudo")
    parser.add_argument("--robot-ip", default="10.21.41.1")
    parser.add_argument("--sdk-rate", type=int, default=200)
    parser.add_argument("--joy-port", type=int, default=9999)
    parser.add_argument("--joy-device", default="/dev/input/js0")
    parser.add_argument("--joy-rate", type=float, default=200.0)
    parser.add_argument("--cache-ttl-sec", type=float, default=8.0)
    parser.add_argument("--cache-radius-m", type=float, default=3.0)
    parser.add_argument("--scan-cache-ttl-sec", type=float, default=0.5, help="short per-direction scan cache TTL for one-sided lidar dropouts")
    parser.add_argument("--scan-cache-min-valid-bins", type=int, default=24, help="minimum visible bins per scan direction before updating the scan cache")
    parser.add_argument("--legacy-106-perception", action="store_true", help="use old 106 noisy_elevation_node path; default uses official /height_map on 103")
    parser.add_argument("--official-height-map-mode", choices=["official_centered", "official_relative", "terrain_z_base"], default="official_centered", help="height_scan conversion for official 103 /height_map")
    parser.add_argument("--height-invalid-mode", choices=["zero", "previous"], default="previous", help="fill invalid height_scan samples with zero or previous valid samples")
    parser.add_argument("--legacy-scan", action=argparse.BooleanOptionalAction, default=True, help="start 103 lidar_to_scan_cpp for legacy 992-dim platform/crawl policies")
    parser.add_argument("--scan-lidar-topic", default="/CLOUD_REGISTERED_BODY")
    parser.add_argument("--web-visualizer", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--visual-remote", default="", help="remote host for web visualizer; default host103")
    parser.add_argument("--visual-jump", default="", help="jump host for web visualizer; default none because official /height_map is on 103")
    parser.add_argument("--visual-height-topic", default="/height_map")
    parser.add_argument("--visual-rate", type=float, default=3.0)
    parser.add_argument("--visual-stride", type=int, default=1)
    parser.add_argument("--visual-port", type=int, default=8765)
    parser.add_argument("--wait-control-sec", type=float, default=20.0)
    parser.add_argument("--lio-start-timeout-sec", type=float, default=12.0)
    parser.add_argument("--lio-start-attempts", type=int, default=4)
    parser.add_argument("--lio-recover-attempts", type=int, default=3)
    parser.add_argument("--height-frame-attempts", type=int, default=2)
    parser.add_argument("--height-enable-timeout-sec", type=float, default=10.0)
    parser.add_argument("--sdk-mode-timeout-sec", type=float, default=20.0)
    parser.add_argument("--sdk-mode-attempts", type=int, default=3)
    parser.add_argument("--start-103-timeout-sec", type=float, default=160.0)
    parser.add_argument("--sync-remote-scripts", action=argparse.BooleanOptionalAction, default=True, help="copy Python perception helpers to the robot before remote preflight")
    parser.add_argument("--skip-preflight", action="store_true")
    parser.add_argument("--no-open-browser", action="store_true")
    parser.add_argument("--tag", default="")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.ask_sudo_password:
        args.sudo_password = getpass.getpass("103/106 sudo password: ")
    sudo_script.password = args.sudo_password
    if args.command == "start":
        start(args)
    elif args.command == "stop":
        stop(args)
    else:
        status(args)


if __name__ == "__main__":
    main()
