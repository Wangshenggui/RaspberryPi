#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header

class EKF:
    def __init__(self):
        # 状态向量 [x, y, theta]
        self.x = np.zeros((3,1))
        # 状态协方差矩阵
        self.P = np.eye(3) * 0.1
        # 过程噪声协方差
        self.Q = np.diag([0.01, 0.01, 0.01])
        # 观测噪声协方差
        self.R = np.diag([0.1, 0.1, 0.1])

    def predict(self, linear_vel, angular_vel, dt):
        theta = self.x[2,0]
        # 运动模型预测
        dx = linear_vel * np.cos(theta) * dt
        dy = linear_vel * np.sin(theta) * dt
        dtheta = angular_vel * dt

        # 状态预测
        self.x[0,0] += dx
        self.x[1,0] += dy
        self.x[2,0] += dtheta
        self.x[2,0] = self.normalize_angle(self.x[2,0])

        # 雅可比矩阵
        F = np.array([
            [1, 0, -linear_vel * np.sin(theta) * dt],
            [0, 1,  linear_vel * np.cos(theta) * dt],
            [0, 0, 1]
        ])

        # 协方差预测
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        # z是雷达测量的位姿 [x, y, theta]
        H = np.eye(3)  # 观测矩阵，直接观测状态
        y = z.reshape((3,1)) - H @ self.x
        y[2,0] = self.normalize_angle(y[2,0])

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2,0] = self.normalize_angle(self.x[2,0])
        self.P = (np.eye(3) - K @ H) @ self.P

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        return angle

class ScanEKFNode:
    def __init__(self):
        rospy.init_node('scan_ekf_node')

        self.ekf = EKF()

        self.last_scan = None
        self.last_scan_pose = None  # 用雷达数据计算的简单位姿，示范用
        self.last_imu_time = None
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0

        self.scan_pub = rospy.Publisher('/scan_new', LaserScan, queue_size=10)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def imu_callback(self, imu_msg):
        current_time = imu_msg.header.stamp.to_sec()
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return

        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time

        # 简单提取线速度和角速度(这里示范只用z轴角速度)
        # 真实应用中需要更复杂处理，比如姿态积分获得速度
        angular_vel = imu_msg.angular_velocity.z

        # 线速度暂用上一刻估计速度，或者0 (这里简化处理)
        linear_vel = self.last_linear_vel

        self.ekf.predict(linear_vel, angular_vel, dt)

    def scan_pose_from_scan(self, scan_msg):
        # 简单示范：雷达位姿用点云质心作为x,y，theta固定0
        ranges = np.array(scan_msg.ranges)
        valid = np.isfinite(ranges)
        if np.sum(valid) == 0:
            return None
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])
        x_mean = np.mean(xs)
        y_mean = np.mean(ys)
        theta = 0.0
        return np.array([x_mean, y_mean, theta])

    def is_scan_changed(self, scan1, scan2, threshold=0.05):
        # 计算两帧质心距离差
        pose1 = self.scan_pose_from_scan(scan1)
        pose2 = self.scan_pose_from_scan(scan2)
        if pose1 is None or pose2 is None:
            return True
        dist = np.linalg.norm(pose1[:2] - pose2[:2])
        return dist > threshold

    def generate_scan_from_ekf(self, last_scan):
        # 简单示范，平移原雷达点云到EKF估计位置
        pose = self.ekf.x.flatten()
        ranges = np.array(last_scan.ranges)
        angles = np.linspace(last_scan.angle_min, last_scan.angle_max, len(ranges))
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        # 旋转和位移点云（只平移x,y，忽略theta旋转简化）
        xs_new = xs + pose[0]
        ys_new = ys + pose[1]

        # 重新计算ranges
        ranges_new = np.sqrt(xs_new**2 + ys_new**2)

        new_scan = LaserScan()
        new_scan.header = Header()
        new_scan.header.stamp = rospy.Time.now()
        new_scan.header.frame_id = last_scan.header.frame_id
        new_scan.angle_min = last_scan.angle_min
        new_scan.angle_max = last_scan.angle_max
        new_scan.angle_increment = last_scan.angle_increment
        new_scan.time_increment = last_scan.time_increment
        new_scan.scan_time = last_scan.scan_time
        new_scan.range_min = last_scan.range_min
        new_scan.range_max = last_scan.range_max
        new_scan.ranges = ranges_new.tolist()
        new_scan.intensities = last_scan.intensities

        return new_scan

    def scan_callback(self, scan_msg):
        if self.last_scan is None:
            self.last_scan = scan_msg
            self.last_scan_pose = self.scan_pose_from_scan(scan_msg)
            self.scan_pub.publish(scan_msg)
            return

        if self.is_scan_changed(scan_msg, self.last_scan):
            pose = self.scan_pose_from_scan(scan_msg)
            if pose is not None:
                self.ekf.update(pose)
                self.last_linear_vel = np.linalg.norm(self.ekf.x[:2])
            self.last_scan = scan_msg
            self.scan_pub.publish(scan_msg)
        else:
            new_scan = self.generate_scan_from_ekf(self.last_scan)
            self.scan_pub.publish(new_scan)

if __name__ == '__main__':
    try:
        node = ScanEKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
