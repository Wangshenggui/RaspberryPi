#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import numpy as np

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

def quaternion_to_rotation_matrix(q):
    # q = [x,y,z,w]
    x, y, z, w = q
    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),         2*(x*z + y*w)],
        [2*(x*y + z*w),           1 - 2*(x**2 + z**2),   2*(y*z - x*w)],
        [2*(x*z - y*w),           2*(y*z + x*w),         1 - 2*(x**2 + y**2)]
    ])
    return R

def small_angle_quaternion(omega, dt):
    angle = np.linalg.norm(omega) * dt
    if angle < 1e-8:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = omega / np.linalg.norm(omega)
    qw = np.cos(angle / 2.0)
    qx, qy, qz = axis * np.sin(angle / 2.0)
    return np.array([qx, qy, qz, qw])

class EKF_SLAM:
    def __init__(self):
        rospy.init_node('ekf_slam_node')

        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.pose_sub = rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_callback)
        self.fused_pose_pub = rospy.Publisher('/slam_pose_new', PoseStamped, queue_size=10)

        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # xyzw

        self.P = np.eye(9) * 0.01
        self.last_imu_time = None
        self.g = np.array([0.0, 0.0, -9.81])
        self.R = np.eye(3) * 0.05
        self.Q = np.eye(9) * 0.01
        self.initialized = False

        # 低通滤波参数
        self.filtered_position = np.zeros(3)
        self.filtered_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.alpha = 0.3  # 滤波系数，0-1之间，越小越平滑
        self.filtered_initialized = False

    def imu_callback(self, msg):
        current_time = msg.header.stamp.to_sec()
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return
        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time

        if dt <= 0 or dt > 0.2:
            return

        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        omega = np.array([wx, wy, wz])
        acc = np.array([ax, ay, az])

        if not self.initialized:
            return

        self.ekf_predict(omega, acc, dt)
        self.publish_pose(msg.header.stamp)

    def pose_callback(self, msg):
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z
        pos_measured = np.array([px, py, pz])

        if not self.initialized:
            self.position = pos_measured
            q = msg.pose.orientation
            self.orientation = np.array([q.x, q.y, q.z, q.w])
            self.velocity = np.zeros(3)
            self.initialized = True
            rospy.loginfo("EKF Initialized with first pose.")
            return

        self.ekf_update(pos_measured)

    def ekf_predict(self, omega, acc, dt):
        dq = small_angle_quaternion(omega, dt)
        self.orientation = quaternion_multiply(self.orientation, dq)
        self.orientation = normalize_quaternion(self.orientation)

        R = quaternion_to_rotation_matrix(self.orientation)
        acc_world = np.dot(R, acc) + self.g

        self.velocity += acc_world * dt
        self.position += self.velocity * dt + 0.5 * acc_world * dt * dt

        F = np.eye(9)
        F[0:3, 3:6] = np.eye(3) * dt

        self.P = F @ self.P @ F.T + self.Q

    def ekf_update(self, pos_measured):
        H = np.zeros((3,9))
        H[:,0:3] = np.eye(3)

        z = pos_measured
        x_pos = self.position

        y = z - x_pos

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        delta_x = K @ y

        self.position += delta_x[0:3]
        self.velocity += delta_x[3:6]

        delta_theta = delta_x[6:9]
        dq = small_angle_quaternion(delta_theta, 1.0)
        self.orientation = quaternion_multiply(self.orientation, dq)
        self.orientation = normalize_quaternion(self.orientation)

        I = np.eye(9)
        self.P = (I - K @ H) @ self.P

    def low_pass_filter(self, new_pos, new_ori):
        if not self.filtered_initialized:
            self.filtered_position = new_pos
            self.filtered_orientation = new_ori
            self.filtered_initialized = True
            return new_pos, new_ori

        self.filtered_position = self.alpha * new_pos + (1 - self.alpha) * self.filtered_position

        self.filtered_orientation = self.alpha * new_ori + (1 - self.alpha) * self.filtered_orientation
        self.filtered_orientation = normalize_quaternion(self.filtered_orientation)

        return self.filtered_position, self.filtered_orientation

    def publish_pose(self, stamp):
        filt_pos, filt_ori = self.low_pass_filter(self.position, self.orientation)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = filt_pos[0]
        pose_msg.pose.position.y = filt_pos[1]
        pose_msg.pose.position.z = filt_pos[2]

        pose_msg.pose.orientation.x = filt_ori[0]
        pose_msg.pose.orientation.y = filt_ori[1]
        pose_msg.pose.orientation.z = filt_ori[2]
        pose_msg.pose.orientation.w = filt_ori[3]

        self.fused_pose_pub.publish(pose_msg)

if __name__ == '__main__':
    ekf_node = EKF_SLAM()
    rospy.spin()
