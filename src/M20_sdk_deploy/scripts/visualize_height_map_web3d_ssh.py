#!/usr/bin/env python3
"""Browser-based 3D-ish height-map visualizer streamed from the perception PC.

This uses only Python stdlib locally.  The remote side still uses ROS 2 Python
packages on the perception computer to sample `/height_map` and sends compact
JSON frames over SSH.  The local web page renders an isometric height surface on
an HTML canvas and can be opened in any browser.
"""

import argparse
import base64
import json
import os
import signal
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


REMOTE_CODE = r"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Streamer(Node):
    def __init__(self, args):
        super().__init__("height_map_web3d_ssh_streamer")
        self.args = args
        self.period = 1.0 / max(args.rate, 0.1)
        self.last_sent = 0.0
        self.create_subscription(PointCloud2, args.height_topic, self.cb, 10)

    def cb(self, msg):
        now = time.time()
        if now - self.last_sent < self.period:
            return
        self.last_sent = now
        pts = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False))
        if not pts:
            return
        arr = np.asarray(pts)
        if arr.dtype.names is not None:
            x = arr["x"].astype(np.float32)
            y = arr["y"].astype(np.float32)
            z = arr["z"].astype(np.float32)
        else:
            arr = arr.astype(np.float32)
            x, y, z = arr[:, 0], arr[:, 1], arr[:, 2]
        finite_xy = np.isfinite(x) & np.isfinite(y)
        if not np.any(finite_xy):
            return
        x, y, z = x[finite_xy], y[finite_xy], z[finite_xy]

        ux = np.unique(np.round(x, 4))
        uy = np.unique(np.round(y, 4))
        rx = float(np.median(np.diff(ux))) if ux.size > 1 else 0.1
        ry = float(np.median(np.diff(uy))) if uy.size > 1 else rx
        res = rx if math.isfinite(rx) and rx > 1e-6 else 0.1
        if math.isfinite(ry) and ry > 1e-6:
            res = min(res, ry)
        x0 = float(np.nanmin(x))
        y0 = float(np.nanmin(y))
        gx = np.rint((x - x0) / res).astype(np.int32)
        gy = np.rint((y - y0) / res).astype(np.int32)
        width = int(gx.max()) + 1
        height = int(gy.max()) + 1
        grid = np.full((height, width), np.nan, dtype=np.float32)
        ok = (gx >= 0) & (gy >= 0) & (gx < width) & (gy < height)
        grid[gy[ok], gx[ok]] = z[ok]
        finite_z = np.isfinite(grid)
        if not np.any(finite_z):
            return
        step = max(1, int(self.args.stride))
        small = grid[::step, ::step]
        obj = {
            "stamp": now,
            "width": int(small.shape[1]),
            "height": int(small.shape[0]),
            "source_width": width,
            "source_height": height,
            "stride": step,
            "res": float(res * step),
            "x_min": x0,
            "y_min": y0,
            "finite": int(np.isfinite(small).sum()),
            "z_min": float(np.nanmin(small)),
            "z_max": float(np.nanmax(small)),
            "z_mean": float(np.nanmean(small)),
            "z": small.tolist(),
        }
        print(json.dumps(obj, separators=(",", ":")), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--height-topic", default="/height_map")
    parser.add_argument("--rate", type=float, default=3.0)
    parser.add_argument("--stride", type=int, default=2)
    args = parser.parse_args()
    rclpy.init()
    node = Streamer(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
"""


HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>M20 Height Map 3D</title>
  <style>
    html, body { margin: 0; height: 100%; background: #111; color: #eee; font-family: sans-serif; }
    #bar { position: fixed; left: 12px; top: 10px; z-index: 2; background: rgba(0,0,0,.55); padding: 8px 10px; border-radius: 6px; font-size: 13px; }
    #bar label { margin-left: 10px; }
    canvas { width: 100vw; height: 100vh; display: block; }
  </style>
</head>
<body>
<div id="bar">
  <span id="status">waiting...</span>
  <label>z-scale <input id="zscale" type="range" min="1" max="12" step="0.5" value="5"></label>
  <label>yaw <input id="yaw" type="range" min="-180" max="180" step="1" value="-45"></label>
  <label>pitch <input id="pitch" type="range" min="10" max="70" step="1" value="35"></label>
</div>
<canvas id="c"></canvas>
<script>
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
const zscaleEl = document.getElementById('zscale');
const yawEl = document.getElementById('yaw');
const pitchEl = document.getElementById('pitch');
let frame = null;

function resize() {
  canvas.width = Math.floor(window.innerWidth * devicePixelRatio);
  canvas.height = Math.floor(window.innerHeight * devicePixelRatio);
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  draw();
}
window.addEventListener('resize', resize);

function color(t) {
  t = Math.max(0, Math.min(1, t));
  const r = Math.floor(60 + 195 * t);
  const g = Math.floor(35 + 95 * (1 - Math.abs(t - .55) * 1.8));
  const b = Math.floor(30 + 55 * (1 - t));
  return `rgb(${r},${g},${b})`;
}

function project(x, y, z, yaw, pitch, scale, cx, cy) {
  const cyaw = Math.cos(yaw), syaw = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const xr = x * cyaw - y * syaw;
  const yr = x * syaw + y * cyaw;
  const ys = yr * cp - z * sp;
  return [cx + xr * scale, cy + ys * scale];
}

function draw() {
  const w = window.innerWidth, h = window.innerHeight;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#111';
  ctx.fillRect(0, 0, w, h);
  if (!frame) return;

  const rows = frame.height, cols = frame.width;
  const z = frame.z;
  const zMin = frame.z_min, zMax = frame.z_max;
  const zRange = Math.max(1e-6, zMax - zMin);
  const yaw = Number(yawEl.value) * Math.PI / 180;
  const pitch = Number(pitchEl.value) * Math.PI / 180;
  const zScale = Number(zscaleEl.value);
  const scale = Math.min(w / (cols * 1.25), h / (rows * .85));
  const cx = w * 0.50, cy = h * 0.58;

  const polys = [];
  for (let iy = 0; iy < rows - 1; iy++) {
    for (let ix = 0; ix < cols - 1; ix++) {
      const vals = [z[iy][ix], z[iy][ix+1], z[iy+1][ix+1], z[iy+1][ix]];
      if (vals.some(v => !Number.isFinite(v))) continue;
      const pts3 = [
        [ix - cols/2, iy - rows/2, vals[0] * zScale],
        [ix + 1 - cols/2, iy - rows/2, vals[1] * zScale],
        [ix + 1 - cols/2, iy + 1 - rows/2, vals[2] * zScale],
        [ix - cols/2, iy + 1 - rows/2, vals[3] * zScale],
      ];
      const avgZ = vals.reduce((a,b) => a + b, 0) / 4;
      const depth = pts3.reduce((a,p) => a + (p[0] * Math.sin(yaw) + p[1] * Math.cos(yaw)), 0) / 4;
      polys.push({pts3, avgZ, depth});
    }
  }
  polys.sort((a, b) => a.depth - b.depth);

  ctx.lineWidth = 0.4;
  for (const poly of polys) {
    const pts = poly.pts3.map(p => project(p[0], p[1], p[2], yaw, pitch, scale, cx, cy));
    const t = (poly.avgZ - zMin) / zRange;
    ctx.beginPath();
    ctx.moveTo(pts[0][0], pts[0][1]);
    for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
    ctx.closePath();
    ctx.fillStyle = color(t);
    ctx.fill();
    ctx.strokeStyle = 'rgba(255,255,255,.16)';
    ctx.stroke();
  }

  statusEl.textContent =
    `${frame.source_height}x${frame.source_width} -> ${rows}x${cols}, ` +
    `z ${zMin.toFixed(3)}..${zMax.toFixed(3)}, mean ${frame.z_mean.toFixed(3)}, ` +
    `stride ${frame.stride}`;
}

[zscaleEl, yawEl, pitchEl].forEach(el => el.addEventListener('input', draw));
new EventSource('/events').onmessage = (ev) => { frame = JSON.parse(ev.data); draw(); };
resize();
</script>
</body>
</html>
"""


class Shared:
    def __init__(self):
        self.cond = threading.Condition()
        self.frame = None
        self.seq = 0

    def set(self, frame):
        with self.cond:
            self.frame = frame
            self.seq += 1
            self.cond.notify_all()


def sh_quote(text):
    return "'" + str(text).replace("'", "'\"'\"'") + "'"


def build_ssh_command(args):
    encoded = base64.b64encode(REMOTE_CODE.encode("utf-8")).decode("ascii")
    remote_args = [
        "--height-topic", args.height_topic,
        "--rate", str(args.rate),
        "--stride", str(args.stride),
    ]
    inner_cmd = (
        "source /opt/ros/foxy/setup.bash && "
        "source /opt/robot/scripts/setup_ros2.sh && "
        f"python3 -u -c \"import base64; exec(base64.b64decode('{encoded}').decode())\" "
        + " ".join(sh_quote(x) for x in remote_args)
    )
    if args.sudo:
        remote_cmd = (
            f"printf {sh_quote(args.sudo_password + chr(10))} | "
            f"sudo -S -p '' bash -lc {sh_quote(inner_cmd)}"
        )
    else:
        remote_cmd = inner_cmd
    cmd = ["ssh", "-o", "StrictHostKeyChecking=no", "-o", f"UserKnownHostsFile={args.known_hosts}"]
    if args.jump:
        cmd += ["-J", args.jump]
    cmd += [args.remote, remote_cmd]
    return cmd


def start_reader(args, shared):
    proc = subprocess.Popen(
        build_ssh_command(args),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )

    def stderr_reader():
        for line in proc.stderr:
            print("[ssh]", line.rstrip(), file=sys.stderr, flush=True)

    threading.Thread(target=stderr_reader, daemon=True).start()
    for line in proc.stdout:
        try:
            shared.set(json.loads(line))
        except json.JSONDecodeError:
            print(line.rstrip(), file=sys.stderr, flush=True)
    raise RuntimeError(f"SSH stream ended with code {proc.poll()}")


def make_handler(shared):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/index"):
                data = HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if self.path.startswith("/events"):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                seq = -1
                while True:
                    with shared.cond:
                        shared.cond.wait_for(lambda: shared.seq != seq, timeout=10)
                        seq = shared.seq
                        frame = shared.frame
                    if frame is None:
                        continue
                    payload = f"data: {json.dumps(frame, separators=(',', ':'))}\n\n".encode("utf-8")
                    try:
                        self.wfile.write(payload)
                        self.wfile.flush()
                    except BrokenPipeError:
                        return
                return
            self.send_error(404)
    return Handler


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--remote", default="user@10.21.33.106")
    parser.add_argument("--jump", default="user@10.21.41.1")
    parser.add_argument("--known-hosts", default="/tmp/known_hosts_106_via_103")
    parser.add_argument("--height-topic", default="/height_map")
    parser.add_argument("--rate", type=float, default=3.0)
    parser.add_argument("--stride", type=int, default=2, help="downsample grid for smoother browser rendering")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--sudo", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sudo-password", default=os.environ.get("M20_SUDO_PASSWORD", "'"))
    parser.add_argument("--no-open", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.jump == "":
        args.jump = None
    shared = Shared()
    reader = threading.Thread(target=start_reader, args=(args, shared), daemon=True)
    reader.start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(shared))
    url = f"http://127.0.0.1:{args.port}/"
    print(f"height-map web3d: {url}", flush=True)
    if not args.no_open:
        webbrowser.open(url)

    def stop(_signum=None, _frame=None):
        server.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    server.serve_forever()


if __name__ == "__main__":
    main()
