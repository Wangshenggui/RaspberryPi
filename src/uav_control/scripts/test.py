#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

def main():
    # 初始化 ROS 节点
    rospy.init_node('my_python_node', anonymous=True)

    # 设置循环频率为 1 Hz（每秒一次）
    rate = rospy.Rate(2)  # 1 Hz

    # 持续循环，直到节点关闭
    while not rospy.is_shutdown():
        # 每秒打印一次信息
        rospy.loginfo("niganmaaiyo>>>>>>>>>>>>>>>>>>>>")

        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    main()
