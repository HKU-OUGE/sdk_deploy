#!/usr/bin/env python3
"""
Scan 盲区实时监控工具
订阅 /scan/multi_layer_features_array，实时显示各扇区盲区触发情况。

Scan 结构：6 俯仰层 × 21 横向bin = 126 (前向) + 126 (后向) = 252
用法：
    ros2 run M20_sdk_deploy scan_blind_monitor.py
    ros2 run M20_sdk_deploy scan_blind_monitor.py --ros-args -p threshold:=0.5
    ros2 run M20_sdk_deploy scan_blind_monitor.py --ros-args -p threshold:=0.5 -p refresh_rate:=5.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import os
import time
import sys

PITCH_LABELS = [
    "Layer 0  -25° 陡俯",
    "Layer 1  -15° 浅俯",
    "Layer 2   -5° 近平↓",
    "Layer 3   +5° 近平↑",
    "Layer 4  +15° 浅仰",
    "Layer 5  +25° 陡仰",
]

NUM_LAYERS = 6
NUM_RAYS = 21
MAX_RANGE = 5.0


class ScanBlindMonitor(Node):
    def __init__(self):
        super().__init__("scan_blind_monitor")

        self.declare_parameter("threshold", 0.2)
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
        if len(msg.data) != 252:
            return
        self.fwd_data = list(msg.data[:126])
        self.bwd_data = list(msg.data[126:])
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

                    line1 = "  Bins  0-10: " + " ".join(bin_strs[:11])
                    line2 = "  Bins 11-20: " + " ".join(bin_strs[11:])
                    # raw print without ljust (ANSI codes break padding)
                    print("│" + line1)
                    print("│" + line2)
                    print("│" + " " * W + "│")

            print("│" + f"  Legend: \033[91mRED\033[0m=blind(<{self.threshold:.2f}m)  \033[92mGREEN\033[0m=valid  \033[90mGRAY\033[0m=max range(5.0m)".ljust(W) + "│")
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
