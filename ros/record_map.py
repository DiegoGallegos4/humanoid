#!/usr/bin/env python3
"""Drive an exploration pattern and record how the SLAM map builds.

Logs, at ~10 Hz: the robot's SLAM pose (map->base_link TF), the latest lidar
scan, and every /map update. Saved to simulation/mujoco/map_log.pkl; render it
on the Mac with simulation/mujoco/render_map.py (PNG + MP4). Run with bringup +
slam already up (see ros/record_map.sh).
"""
import math
import pickle
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

OUT = "/nori/simulation/mujoco/map_log.pkl"
LIDAR_X = 0.12  # lidar_link offset ahead of base_link (URDF)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Node("nori_record_map")
    pub = node.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    maps, last = [], {}
    t0 = time.time()

    def on_map(m):
        maps.append({
            "t": time.time() - t0,
            "w": m.info.width, "h": m.info.height, "res": m.info.resolution,
            "ox": m.info.origin.position.x, "oy": m.info.origin.position.y,
            "data": np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width),
        })

    node.create_subscription(OccupancyGrid, "/map", on_map, qos)
    node.create_subscription(LaserScan, "/scan", lambda s: last.update(scan=s),
                             rclpy.qos.qos_profile_sensor_data)
    node.create_subscription(Odometry, "/ground_truth/odom",
                             lambda m: last.update(gt=m.pose.pose),
                             rclpy.qos.qos_profile_sensor_data)
    tf_buf = Buffer()
    TransformListener(tf_buf, node)
    frames = []

    def sample():
        try:
            tf = tf_buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:  # TF not ready yet
            return
        x, y = tf.transform.translation.x, tf.transform.translation.y
        th = yaw_of(tf.transform.rotation)
        pts = np.zeros((0, 2))
        s = last.get("scan")
        if s is not None:
            r = np.array(s.ranges)
            a = s.angle_min + np.arange(len(r)) * s.angle_increment
            ok = np.isfinite(r)
            lx = LIDAR_X + r[ok] * np.cos(a[ok])     # base frame
            ly = r[ok] * np.sin(a[ok])
            pts = np.stack([x + lx * math.cos(th) - ly * math.sin(th),
                            y + lx * math.sin(th) + ly * math.cos(th)], axis=1)
        gt = last.get("gt")
        gt_pose = (gt.position.x, gt.position.y, yaw_of(gt.orientation)) if gt else None
        try:
            to = tf_buf.lookup_transform("odom", "base_link", rclpy.time.Time())
            odom_pose = (to.transform.translation.x, to.transform.translation.y,
                         yaw_of(to.transform.rotation))
        except Exception:
            odom_pose = None
        frames.append({"t": time.time() - t0, "pose": (x, y, th), "scan": pts,
                       "gt": gt_pose, "odom": odom_pose})

    def drive(lin, ang, dt):
        end = time.time() + dt
        next_sample = time.time()
        while time.time() < end:
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.twist.linear.x = lin
            msg.twist.angular.z = ang
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.time() >= next_sample:
                sample()
                next_sample += 0.1

    # same exploration as the SLAM gate: spin, trace a square, spin again
    drive(0.0, 0.0, 1.0)
    drive(0.0, 0.6, 11.0)
    for _ in range(4):
        drive(0.25, 0.0, 3.0)
        drive(0.0, 0.9, 1.8)
    drive(0.0, 0.6, 11.0)
    drive(0.0, 0.0, 2.0)

    with open(OUT, "wb") as f:
        pickle.dump({"frames": frames, "maps": maps}, f)
    print(f"recorded {len(frames)} pose frames, {len(maps)} map updates -> {OUT}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
