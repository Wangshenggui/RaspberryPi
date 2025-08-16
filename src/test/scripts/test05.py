#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import numpy as np

# --- quaternion helpers ---
def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    return q / norm if norm > 0 else q


def quaternion_multiply(q1, q2):
    # q = [x,y,z,w]
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([x, y, z, w])


def quaternion_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quaternion_to_rotation_matrix(q):
    # q is [x, y, z, w]
    x, y, z, w = q
    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),         2*(x*z + y*w)],
        [2*(x*y + z*w),           1 - 2*(x**2 + z**2),   2*(y*z - x*w)],
        [2*(x*z - y*w),           2*(y*z + x*w),         1 - 2*(x**2 + y**2)]
    ])
    return R


def small_angle_quaternion(omega, dt):
    # omega: 3-vector angular velocity or small rotation vector
    angle = np.linalg.norm(omega) * dt
    if angle < 1e-8:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = omega / np.linalg.norm(omega)
    qw = np.cos(angle / 2.0)
    qx, qy, qz = axis * np.sin(angle / 2.0)
    return np.array([qx, qy, qz, qw])


class EKF_SLAM:
    def __init__(self):
        # rospy.init_node('ekf_slam_node')
        rospy.init_node('ekf_slam_node', log_level=rospy.DEBUG)


        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.pose_sub = rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_callback)
        self.fused_pose_pub = rospy.Publisher('/ikun', PoseStamped, queue_size=10)

        # 状态: [pos(3), vel(3), ori_err(3), gyro_bias(3), acc_bias(3)] -> 15
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # xyzw quaternion

        self.gyro_bias = np.zeros(3)
        self.acc_bias = np.zeros(3)

        self.P = np.eye(15) * 0.01

        self.last_imu_time = None

        # 注意坐标系: 这里假设 Z 向上为正（IMU 原始加速度包含重力指向）
        # 如果你的 IMU 报告 Z 向上为正，则重力在世界坐标应为 [0,0,-9.81]
        self.g = np.array([0.0, 0.0, -9.81])

        # 过程噪声 Q
        q_pos = 0.001
        q_vel = 0.01
        q_ori = 0.001
        q_gyro_bias = 1e-6
        q_acc_bias = 1e-5
        self.Q = np.diag([
            q_pos, q_pos, q_pos,
            q_vel, q_vel, q_vel,
            q_ori, q_ori, q_ori,
            q_gyro_bias, q_gyro_bias, q_gyro_bias,
            q_acc_bias, q_acc_bias, q_acc_bias
        ])

        # 测量噪声 R (position + orientation_error)
        self.R_pos = np.eye(3) * 0.05
        self.R_ori = np.eye(3) * 0.02
        self.R = np.block([
            [self.R_pos, np.zeros((3,3))],
            [np.zeros((3,3)), self.R_ori]
        ])

        self.initialized = False

        # 低通
        self.filtered_position = np.zeros(3)
        self.filtered_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.alpha = 0.3
        self.filtered_initialized = False

    def imu_callback(self, msg):
        current_time = msg.header.stamp.to_sec()
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return
        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time

        if dt <= 0 or dt > 0.2:
            rospy.logwarn(f"Bad dt: {dt}")
            return

        if not self.initialized:
            return

        omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]) - self.gyro_bias

        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]) - self.acc_bias

        self.ekf_predict(omega, acc, dt)
        self.publish_pose(msg.header.stamp)

    def pose_callback(self, msg):
        pos_measured = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        ori_measured = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        if not self.initialized:
            self.position = pos_measured.copy()
            self.orientation = ori_measured.copy()
            self.velocity = np.zeros(3)
            self.gyro_bias = np.zeros(3)
            self.acc_bias = np.zeros(3)
            self.P = np.eye(15) * 0.01
            self.initialized = True

            self.filtered_position = pos_measured.copy()
            self.filtered_orientation = ori_measured.copy()
            self.filtered_initialized = True

            rospy.loginfo("EKF initialized.")
            return

        # 使用位置 + 姿态观测进行更新
        self.ekf_update(pos_measured, ori_measured)

        filt_pos, filt_ori = self.low_pass_filter(self.position, self.orientation)
        self.publish_pose_with_data(filt_pos, filt_ori, msg.header.stamp)

    def ekf_predict(self, omega, acc, dt):
        # 更新姿态（积分陀螺仪角速度）
        dq = small_angle_quaternion(omega, dt)
        self.orientation = quaternion_multiply(self.orientation, dq)
        self.orientation = normalize_quaternion(self.orientation)

        # 把加速度转换到世界坐标并补偿重力
        Rm = quaternion_to_rotation_matrix(self.orientation)
        acc_world = Rm.dot(acc) + self.g

        # 状态更新
        self.position += self.velocity * dt + 0.5 * acc_world * dt * dt
        self.velocity += acc_world * dt

        # 构造状态转移矩阵 F
        F = np.eye(15)
        F[0:3, 3:6] = np.eye(3) * dt
        # 速度对姿态误差的影响 (近似)
        F[3:6, 6:9] = -self.skew(acc_world) * dt
        # 速度对加速度偏置的影响
        F[3:6, 12:15] = -Rm * dt
        # 姿态误差随陀螺仪偏置变化
        F[6:9, 9:12] = -np.eye(3) * dt

        # 协方差预测
        self.P = F @ self.P @ F.T + self.Q

    def ekf_update(self, pos_measured, ori_measured):
        # 计算测量剩余: 位置
        z_pos = pos_measured

        # 计算测量剩余: 姿态误差（SLAM 相对于 EKF 的旋转），使用 q_err = q_meas * q_est^{-1}
        q_err = quaternion_multiply(ori_measured, quaternion_conjugate(self.orientation))
        q_err = normalize_quaternion(q_err)
        # 小角近似：旋转向量大约为 2 * vec(q_err) when w ~ 1
        delta_theta_meas = 2.0 * q_err[0:3]

        # 组合观测向量 z (6x1)
        z = np.hstack((z_pos, delta_theta_meas))

        # 预测观测 h(x): 位置由状态直接给出，姿态误差部分预测为 0（因为状态中的姿态误差是局部误差）
        h = np.hstack((self.position, np.zeros(3)))

        # 观测矩阵 H (6x15)
        H = np.zeros((6, 15))
        H[0:3, 0:3] = np.eye(3)   # 位置 -> 状态位置
        H[3:6, 6:9] = np.eye(3)   # 姿态误差 -> 状态 6:9

        # 卡尔曼增益
        y = z - h
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        delta_x = K @ y

        # 应用状态增量
        self.position += delta_x[0:3]
        self.velocity += delta_x[3:6]

        delta_theta = delta_x[6:9]
        dq = small_angle_quaternion(delta_theta, 1.0)
        self.orientation = quaternion_multiply(self.orientation, dq)
        self.orientation = normalize_quaternion(self.orientation)

        self.gyro_bias += delta_x[9:12]
        self.acc_bias += delta_x[12:15]

        # 协方差更新（Joseph form 可选，这里使用简单形式）
        I = np.eye(15)
        self.P = (I - K @ H) @ self.P

    def low_pass_filter(self, position, orientation):
        if not self.filtered_initialized:
            self.filtered_position = position.copy()
            self.filtered_orientation = orientation.copy()
            self.filtered_initialized = True
            return position, orientation

        self.filtered_position = self.alpha * position + (1 - self.alpha) * self.filtered_position

        # 对四元数做线性插值然后归一化（简单且常用）
        q = self.alpha * orientation + (1 - self.alpha) * self.filtered_orientation
        q = normalize_quaternion(q)
        self.filtered_orientation = q

        return self.filtered_position, self.filtered_orientation

    def publish_pose(self, stamp):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]

        pose_msg.pose.orientation.x = self.orientation[0]
        pose_msg.pose.orientation.y = self.orientation[1]
        pose_msg.pose.orientation.z = self.orientation[2]
        pose_msg.pose.orientation.w = self.orientation[3]

        self.fused_pose_pub.publish(pose_msg)

    def publish_pose_with_data(self, pos, ori, stamp):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = pos[0]
        pose_msg.pose.position.y = pos[1]
        pose_msg.pose.position.z = pos[2]
        pose_msg.pose.orientation.x = ori[0]
        pose_msg.pose.orientation.y = ori[1]
        pose_msg.pose.orientation.z = ori[2]
        pose_msg.pose.orientation.w = ori[3]
        self.fused_pose_pub.publish(pose_msg)

    def skew(self, v):
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])


if __name__ == '__main__':
    ekf = EKF_SLAM()
    rospy.spin()
