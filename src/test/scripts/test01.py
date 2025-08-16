#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import numpy as np

def imu_simulator():
    rospy.init_node('imu_simulator')
    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(100)  # 100Hz

    while not rospy.is_shutdown():
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        # 设置虚拟数据（无噪声）
        imu_msg.linear_acceleration.x = 0.0  # m/s²
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # 重力加速度
        imu_msg.angular_velocity.z = 0.1     # 绕Z轴旋转（rad/s）
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import numpy as np

def imu_simulator():
    rospy.init_node('imu_simulator')
    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(100)  # 100Hz

    while not rospy.is_shutdown():
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        # 设置虚拟数据（无噪声）
        imu_msg.linear_acceleration.x = 0.0  # m/s²
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # 重力加速度
        imu_msg.angular_velocity.z = 0     # 绕Z轴旋转（rad/s）

        # 添加噪声（可选）
        imu_msg.linear_acceleration.x += np.random.normal(0, 0.1)
        imu_msg.angular_velocity.z += np.random.normal(0, 0.01)

        pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_simulator()
    except rospy.ROSInterruptException:
        pass
        # 添加噪声（可选）
        imu_msg.linear_acceleration.x += np.random.normal(0, 0.1)
        imu_msg.angular_velocity.z += np.random.normal(0, 0.01)

        pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_simulator()
    except rospy.ROSInterruptException:
        pass