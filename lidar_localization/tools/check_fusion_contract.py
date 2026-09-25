#!/usr/bin/env python3
"""Overí, že /pcl_pose a /alignment_status spĺňajú kontrakt MowgliNext fusion_graph.

Kontroluje to isté, čo prijímacia strana (nie len že topic existuje):
  /pcl_pose         frame_id == "map", vek stampu <= 0.5 s, kovariancia XY/yaw
                    konečná, nenulová, PSD, sigma <= 0.75 m, normovaný quaternion
  /alignment_status status.name obsahuje "lidar_localization_ros2/alignment",
                    kľúče failure_category / consecutive_rejected_updates /
                    reinitialization_requested, čerstvosť <= 2 s
  /tf (--tf)        z map/odom smú ísť len map->odom a odom->base_footprint
                    (fusion_graph); čokoľvek iné (napr. map->base_link) = chyba

Použitie (v kontajneri lidar_localization):
  docker exec -it lidar_localization python3 /opt/tools/check_fusion_contract.py --duration 10
"""

import argparse
import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

MAP_FRAME = "map"
MAX_POSE_AGE_SEC = 0.5
MAX_STATUS_AGE_SEC = 2.0
MAX_XY_SIGMA_M = 0.75
STATUS_NAME = "lidar_localization_ros2/alignment"
# jediné map/odom hrany v TF strome, obe od fusion_graph (REP-105)
ALLOWED_TF_EDGES = {(MAP_FRAME, "odom"), ("odom", "base_footprint")}
REQUIRED_KEYS = (
    "failure_category",
    "consecutive_rejected_updates",
    "reinitialization_requested",
)


def check_pose(msg, now_sec):
    """Vráti zoznam porušení kontraktu pre jednu /pcl_pose správu."""
    errors = []
    if msg.header.frame_id != MAP_FRAME:
        errors.append(f"frame_id '{msg.header.frame_id}' != '{MAP_FRAME}'")
    stamp_sec = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
    age = now_sec - stamp_sec
    if age > MAX_POSE_AGE_SEC:
        errors.append(f"stamp starý {age:.3f} s (> {MAX_POSE_AGE_SEC})")
    if age < -0.1:
        errors.append(f"stamp je {-age:.3f} s v budúcnosti (hodiny?)")

    c = msg.pose.covariance
    cxx, cxy, cyx, cyy, cyaw = c[0], c[1], c[6], c[7], c[35]
    if not all(math.isfinite(v) for v in (cxx, cxy, cyx, cyy, cyaw)):
        errors.append("kovariancia nie je konečná")
    else:
        det = cxx * cyy - cxy * cyx
        if cxx <= 0.0 or cyy <= 0.0 or det <= 0.0:
            errors.append(f"XY kovariancia nie je kladne definitná ({cxx}, {cxy}; {cyx}, {cyy})")
        elif math.sqrt(max(cxx, cyy)) > MAX_XY_SIGMA_M:
            errors.append(f"XY sigma {math.sqrt(max(cxx, cyy)):.3f} m > {MAX_XY_SIGMA_M} (bude zahodené)")
        if cyaw <= 0.0:
            errors.append(f"yaw variancia [35] = {cyaw} (nie kladná)")

    q = msg.pose.pose.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if abs(norm - 1.0) > 1e-3:
        errors.append(f"quaternion nie je normovaný (|q| = {norm:.4f})")
    return errors


def check_status(msg):
    """Vráti (porušenia, hodnoty kľúčov) pre jednu /alignment_status správu."""
    matches = [s for s in msg.status if STATUS_NAME in s.name]
    if not matches:
        return [f"žiadny status s menom obsahujúcim '{STATUS_NAME}'"], {}
    values = {kv.key: kv.value for kv in matches[0].values}
    errors = [f"chýba kľúč '{k}'" for k in REQUIRED_KEYS if k not in values]
    if "consecutive_rejected_updates" in values:
        try:
            int(values["consecutive_rejected_updates"])
        except ValueError:
            errors.append("consecutive_rejected_updates nie je integer")
    return errors, {k: values.get(k) for k in REQUIRED_KEYS}


class ContractChecker(Node):
    def __init__(self, watch_tf):
        super().__init__("fusion_contract_checker")
        self.pose_count = 0
        self.pose_errors = {}
        self.last_pose = None
        self.status_count = 0
        self.status_errors = {}
        self.last_status_values = {}
        self.status_receive_times = []
        self.tf_edges = {}
        self.create_subscription(PoseWithCovarianceStamped, "/pcl_pose", self.on_pose, 10)
        self.create_subscription(DiagnosticArray, "/alignment_status", self.on_status, 10)
        if watch_tf:
            self.create_subscription(TFMessage, "/tf", self.on_tf, 100)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg):
        self.pose_count += 1
        self.last_pose = msg
        for err in check_pose(msg, self.now_sec()):
            self.pose_errors[err.split(" (")[0]] = err

    def on_status(self, msg):
        self.status_count += 1
        self.status_receive_times.append(time.monotonic())
        errors, values = check_status(msg)
        self.last_status_values = values
        for err in errors:
            self.status_errors[err] = err

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id in (MAP_FRAME, "odom"):
                self.tf_edges[(t.header.frame_id, t.child_frame_id)] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=10.0, help="ako dlho počúvať [s]")
    parser.add_argument("--tf", action="store_true", help="vypíš aj map->* / odom->* hrany z /tf")
    args = parser.parse_args()

    rclpy.init()
    node = ContractChecker(args.tf)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    ok = True
    print(f"\n/pcl_pose: {node.pose_count} správ za {args.duration:.0f} s")
    if node.pose_count == 0:
        ok = False
        print("  CHYBA: nič neprišlo (mapa načítaná? scany chodia? DDS domain/cyclonedds?)")
    if node.last_pose is not None:
        p = node.last_pose
        c = p.pose.covariance
        q = p.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        print(
            f"  posledná: frame={p.header.frame_id} x={p.pose.pose.position.x:.2f} "
            f"y={p.pose.pose.position.y:.2f} yaw={math.degrees(yaw):.1f} deg "
            f"sigma_xy={math.sqrt(max(c[0], 0.0)):.3f}/{math.sqrt(max(c[7], 0.0)):.3f} m "
            f"sigma_yaw={math.degrees(math.sqrt(max(c[35], 0.0))):.2f} deg"
        )
    for err in node.pose_errors.values():
        ok = False
        print(f"  CHYBA: {err}")

    print(f"/alignment_status: {node.status_count} správ")
    if node.status_count == 0:
        ok = False
        print("  CHYBA: nič neprišlo -> fusion_graph bude /pcl_pose ignorovať (fail-closed)")
    else:
        times = node.status_receive_times
        max_gap = max((b - a for a, b in zip(times, times[1:])), default=0.0)
        print(f"  hodnoty: {node.last_status_values}  max. medzera {max_gap:.2f} s")
        if max_gap > MAX_STATUS_AGE_SEC:
            ok = False
            print(f"  CHYBA: medzera medzi statusmi > {MAX_STATUS_AGE_SEC} s")
    for err in node.status_errors.values():
        ok = False
        print(f"  CHYBA: {err}")

    if args.tf:
        print("/tf hrany z map/odom (majú byť LEN od fusion_graph: map->odom, odom->base_footprint):")
        for parent, child in sorted(node.tf_edges):
            allowed = (parent, child) in ALLOWED_TF_EDGES
            ok = ok and allowed
            print(f"  {parent} -> {child}" + ("" if allowed else "   CHYBA: tento TF nesmie existovať"))

    print("\nOK" if ok else "\nNEPREŠLO")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
