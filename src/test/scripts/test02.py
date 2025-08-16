#!/usr/bin/env python3
import rospy
import numpy as np
import math
from sensor_msgs.msg import Imu, LaserScan
from copy import deepcopy

def wrap_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def scan_to_points(scan_msg, max_range=10.0):
    angles = np.arange(scan_msg.angle_min,
                       scan_msg.angle_max + 1e-9,
                       scan_msg.angle_increment)
    ranges = np.array(scan_msg.ranges[:len(angles)])
    mask = np.isfinite(ranges) & (ranges > scan_msg.range_min) & (ranges < min(scan_msg.range_max, max_range))
    xs = ranges[mask] * np.cos(angles[mask])
    ys = ranges[mask] * np.sin(angles[mask])
    return np.vstack([xs, ys]).T

def pca_yaw(pts):
    if pts.shape[0] < 5:
        return 0.0, False
    mean = pts.mean(axis=0)
    pts_c = pts - mean
    C = np.cov(pts_c.T)
    w, v = np.linalg.eig(C)
    idx = np.argmax(w)
    principal = v[:, idx]
    angle = math.atan2(principal[1], principal[0])
    return wrap_angle(angle), True

def compare_scans(pts1, pts2):
    # 简单比较点集均方差
    if pts1.shape[0] == 0 or pts2.shape[0] == 0:
        return float('inf')
    n = min(len(pts1), len(pts2))
    diff = np.linalg.norm(pts1[:n] - pts2[:n], axis=1)
    return np.mean(diff)

def rotate_scan(scan_msg, yaw):
    new_scan = deepcopy(scan_msg)
    angles = np.arange(scan_msg.angle_min,
                       scan_msg.angle_max + 1e-9,
                       scan_msg.angle_increment)
    ranges = np.array(scan_msg.ranges[:len(angles)])
    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)
    c = math.cos(-yaw)
    s = math.sin(-yaw)
    xr = c * xs - s * ys
    yr = s * xs + c * ys
    new_ranges = np.sqrt(xr**2 + yr**2)
    mask_valid = np.isfinite(ranges) & (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)
    out_ranges = np.full_like(ranges, float('inf'), dtype=float)
    out_ranges[mask_valid] = new_ranges[mask_valid]
    new_scan.ranges = out_ranges.tolist()
    return new_scan

class IMULidarFusion:
    def __init__(self):
        rospy.init_node("imu_lidar_fusion", anonymous=False)
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu")
        self.out_topic  = rospy.get_param("~out_topic", "/scan_new")
        self.diff_threshold = rospy.get_param("~diff_threshold", 0.01)  # m

        self.yaw_est = 0.0
        self.last_time = None
        self.last_scan_pts = None
        self.last_scan_yaw = None

        self.pub_scan = rospy.Publisher(self.out_topic, LaserScan, queue_size=5)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_cb, queue_size=50)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb, queue_size=5)

        rospy.loginfo("IMU-Lidar fusion node started.")
    
    def imu_cb(self, msg: Imu):
        t = msg.header.stamp.to_sec()
        if self.last_time is not None:
            dt = t - self.last_time
            if dt > 0:
                self.yaw_est += msg.angular_velocity.z * dt
                self.yaw_est = wrap_angle(self.yaw_est)
        self.last_time = t

    def scan_cb(self, msg: LaserScan):
        current_pts = scan_to_points(msg)
        current_yaw, valid = pca_yaw(current_pts)

        if self.last_scan_pts is not None and valid:
            diff = compare_scans(self.last_scan_pts, current_pts)
            if diff > self.diff_threshold and self.last_scan_yaw is not None:
                # 用雷达 yaw 修正 yaw_est
                delta_yaw = wrap_angle(current_yaw - self.last_scan_yaw)
                self.yaw_est = wrap_angle(self.yaw_est + delta_yaw)
                rospy.loginfo("Lidar update: Δyaw=%.3f rad (diff=%.3f)", delta_yaw, diff)
            else:
                rospy.loginfo("IMU only (diff=%.3f)", diff)

        self.last_scan_pts = current_pts
        self.last_scan_yaw = current_yaw if valid else None

        corrected_scan = rotate_scan(msg, self.yaw_est)
        self.pub_scan.publish(corrected_scan)

if __name__ == "__main__":
    try:
        node = IMULidarFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
