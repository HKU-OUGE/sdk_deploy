#!/usr/bin/env python3
"""
Scan 盲区实时监控工具 (半球 hemispherical 版本)
订阅 /scan/multi_layer_features_array，实时显示各极角层盲区触发情况。

Scan 结构: 16 polar × 31 azimuth = 496 (前向) + 496 (后向) = 992
  - polar  ∈ [0°, 90°]:  从 boresight 出发的极角 (0 = 正前/正后, 90° = 半球边)
  - azimuth ∈ [-180°, 180°]: 绕 boresight 一圈
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


NUM_LAYERS = 16          # polar bins
NUM_RAYS = 31            # azimuth bins
NUM_PER_DIR = NUM_LAYERS * NUM_RAYS   # 496
NUM_TOTAL = NUM_PER_DIR * 2            # 992
MAX_RANGE = 2.5

PITCH_LABELS = [
    f"L{i:02d}  polar={int(round(i * 90.0 / (NUM_LAYERS - 1)))}°"
    for i in range(NUM_LAYERS)
]


class ScanBlindMonitor(Node):
    def __init__(self):
        super().__init__("scan_blind_monitor")

        self.declare_parameter("threshold", 0.3)
        self.declare_parameter("refresh_rate", 2.0)

        self.threshold = self.get_parameter("threshold").value
        self.refresh_rate = self.get_parameter("refresh_rate").value

        self.sub = self.create_subscription(
            Float32MultiArray,
            "/scan/multi_layer_features_array",
            self.scan_cb,
            10,
        )

        self.timer = self.create_timer(1.0 / self.refresh_rate, self.display)

        self.fwd_data = None
        self.bwd_data = None
        self.msg_count = 0
        self.last_stamp = time.time()

        self.get_logger().info(
            f"Scan Blind Monitor started | threshold={self.threshold:.2f}m | "
            f"refresh={self.refresh_rate:.1f}Hz"
        )

    def scan_cb(self, msg: Float32MultiArray):
        if len(msg.data) != NUM_TOTAL:
            return
        self.fwd_data = list(msg.data[:NUM_PER_DIR])
        self.bwd_data = list(msg.data[NUM_PER_DIR:])
        self.msg_count += 1

    def analyze_sector(self, bins, layer_idx):
        start = layer_idx * NUM_RAYS
        sector = bins[start : start + NUM_RAYS]

        blind_count = sum(1 for v in sector if v < self.threshold)
        max_range_count = sum(1 for v in sector if abs(v - MAX_RANGE) < 0.01)
        valid = [v for v in sector if self.threshold <= v < MAX_RANGE]
        min_dist = min(valid) if valid else MAX_RANGE
        avg_dist = sum(valid) / len(valid) if valid else MAX_RANGE

        return {
            "blind": blind_count,
            "max_range": max_range_count,
            "valid": len(valid),
            "min": min_dist,
            "avg": avg_dist,
        }

    def bar(self, count, total=NUM_RAYS, width=10):
        filled = round(count / total * width) if total > 0 else 0
        return "█" * filled + "░" * (width - filled)

    def display(self):
        if self.fwd_data is None:
            return

        now = time.time()
        hz = self.msg_count / (now - self.last_stamp) if (now - self.last_stamp) > 0 else 0
        self.msg_count = 0
        self.last_stamp = now

        os.system("clear")

        W = 96
        print("┌" + "─" * W + "┐")
        print(
            "│"
            + f"  SCAN BLIND SPOT MONITOR   │   Threshold: {self.threshold:.2f}m"
            f"   │   Scan Rate: {hz:.1f} Hz".ljust(W)
            + "│"
        )
        print("├" + "─" * W + "┤")

        fwd_sectors = []
        bwd_sectors = []
        for i in range(NUM_LAYERS):
            fwd_sectors.append(self.analyze_sector(self.fwd_data, i))
            bwd_sectors.append(self.analyze_sector(self.bwd_data, i))

        fwd_triggered = [i for i in range(NUM_LAYERS) if fwd_sectors[i]["blind"] > 0]
        bwd_triggered = [i for i in range(NUM_LAYERS) if bwd_sectors[i]["blind"] > 0]

        total_fwd_blind = sum(s["blind"] for s in fwd_sectors)
        total_bwd_blind = sum(s["blind"] for s in bwd_sectors)

        status_fwd = f"🔴 {total_fwd_blind} bins" if total_fwd_blind > 0 else "🟢 None"
        status_bwd = f"🔴 {total_bwd_blind} bins" if total_bwd_blind > 0 else "🟢 None"

        print("│" + f"  ▶ FORWARD  blind: {status_fwd}       ◀ BACKWARD  blind: {status_bwd}".ljust(W) + "│")
        print("├" + "─" * W + "┤")

        # header
        hdr = (
            f"  {'Direction':<5} {'Sector':<20} "
            f"{'Blind':>5} {'Valid':>5} {'NoHit':>5} "
            f"{'Min(m)':>7} {'Avg(m)':>7} "
            f"{'Blind Bar':<12} {'Status':<6}"
        )
        print("│" + hdr.ljust(W) + "│")
        print("├" + "─" * W + "┤")

        any_triggered = False

        for direction, sectors, triggered, label in [
            ("FWD", fwd_sectors, fwd_triggered, "▶"),
            ("BWD", bwd_sectors, bwd_triggered, "◀"),
        ]:
            for i in range(NUM_LAYERS):
                s = sectors[i]
                is_hit = s["blind"] > 0
                if is_hit:
                    any_triggered = True
                    tag = "⚠ HIT"
                else:
                    tag = "  OK "

                row = (
                    f"  {label:<5} {PITCH_LABELS[i]:<20} "
                    f"{s['blind']:>5} {s['valid']:>5} {s['max_range']:>5} "
                    f"{s['min']:>7.3f} {s['avg']:>7.3f} "
                    f"{self.bar(s['blind']):<12} {tag:<6}"
                )
                print("│" + row.ljust(W) + "│")

            if direction == "FWD":
                print("│" + " " * W + "│")

        print("├" + "─" * W + "┤")

        # triggered sector details
        if any_triggered:
            print("│" + "  ⚠  TRIGGERED SECTORS DETAIL".ljust(W) + "│")
            print("│" + " " * W + "│")

            for direction, sectors, triggered, label, raw_data in [
                ("FORWARD", fwd_sectors, fwd_triggered, "▶", self.fwd_data),
                ("BACKWARD", bwd_sectors, bwd_triggered, "◀", self.bwd_data),
            ]:
                for i in triggered:
                    s = sectors[i]
                    start = i * NUM_RAYS
                    vals = raw_data[start : start + NUM_RAYS]

                    print("│" + f"  {label} {direction} {PITCH_LABELS[i]}  —  {s['blind']}/{NUM_RAYS} bins blind".ljust(W) + "│")

                    # print bin values in a compact row
                    bin_strs = []
                    for j, v in enumerate(vals):
                        if v < self.threshold:
                            bin_strs.append(f"\033[91m{v:5.2f}\033[0m")
                        elif abs(v - MAX_RANGE) < 0.01:
                            bin_strs.append(f"\033[90m{v:5.2f}\033[0m")
                        else:
                            bin_strs.append(f"\033[92m{v:5.2f}\033[0m")

                    # 31 azimuth bins → 拆 3 行: 0-10, 11-20, 21-30
                    print("│" + "  Bins  0-10: " + " ".join(bin_strs[:11]))
                    print("│" + "  Bins 11-20: " + " ".join(bin_strs[11:21]))
                    print("│" + "  Bins 21-30: " + " ".join(bin_strs[21:]))
                    print("│" + " " * W + "│")

            print("│" + f"  Legend: \033[91mRED\033[0m=blind(<{self.threshold:.2f}m)  \033[92mGREEN\033[0m=valid  \033[90mGRAY\033[0m=max range({MAX_RANGE}m)".ljust(W) + "│")
        else:
            print(
                "│"
                + f"  ✅ No blind spots triggered at threshold={self.threshold:.2f}m"
                   f"  (try increasing threshold, e.g. -p threshold:=0.5)".ljust(W)
                + "│"
            )

        print("├" + "─" * W + "┤")

        # global min distances
        fwd_global_min = min(
            (v for v in self.fwd_data if v > 0), default=MAX_RANGE
        )
        bwd_global_min = min(
            (v for v in self.bwd_data if v > 0), default=MAX_RANGE
        )
        print(
            "│"
            + f"  Global Min Distance:  FWD={fwd_global_min:.3f}m  BWD={bwd_global_min:.3f}m"
              f"      Threshold={self.threshold:.2f}m".ljust(W)
            + "│"
        )
        print("└" + "─" * W + "┘")
        print(
            f"\n  Tip: Adjust threshold with:  "
            f"ros2 run M20_sdk_deploy scan_blind_monitor.py --ros-args -p threshold:=<value>"
        )

        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = ScanBlindMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
