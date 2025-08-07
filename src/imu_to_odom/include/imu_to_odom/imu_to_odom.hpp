#ifndef IMU_TO_ODOM_IMU_TO_ODOM_HPP
#define IMU_TO_ODOM_IMU_TO_ODOM_HPP

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

struct Point
{
    Eigen::Vector3d pos; // 位置
    Eigen::Matrix3d orien; // 姿态 旋转矩阵表示
    Eigen::Vector3d w; // 角速度
    Eigen::Vector3d v; // 线速度
};

// IMU处理类
class ImuOdom
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber imusub_;
    ros::Subscriber gpssub_;
    ros::Publisher odompub_;
    nav_msgs::Odometry odom_;
    ros::Time time_;
    Point point_;
    Eigen::Vector3d gravity_;

    double deltaT_;
    bool firstT_;
    Eigen::Vector3d lastGpsPos_;
    ros::Time lastGpsTime_;
    
public:
    //! Constructor.
    ImuOdom(ros::NodeHandle& nh);
    //! Destructor.
    ~ImuOdom();

    void ImuCallback(const sensor_msgs::Imu& msg);
    void GpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void setGravity(const geometry_msgs::Vector3& msg);
    void calcPosition(const geometry_msgs::Vector3& msg);
    void calcOrientation(const geometry_msgs::Vector3& msg);
    void updateOdom(const Point& point);

    Eigen::Vector3d interpolatePosition(const ros::Time& t_imu, const ros::Time& t_gps,
                                        const Eigen::Vector3d& p_imu, const Eigen::Vector3d& p_gps);
};

ImuOdom::ImuOdom(ros::NodeHandle& nh) : nh_(nh), firstT_(true)
{
    imusub_ = nh_.subscribe("/imu", 32, &ImuOdom::ImuCallback, this);
    gpssub_ = nh_.subscribe("/slam_out_pose", 32, &ImuOdom::GpsCallback, this);
    odompub_ = nh_.advertise<nav_msgs::Odometry>("imu_odom", 32);
    
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_link";
    
    Eigen::Vector3d zero(0, 0, 0);
    point_.pos = zero;
    point_.orien = Eigen::Matrix3d::Identity();
    point_.v = zero;
    point_.w = zero;
    gravity_ = zero;
    
    lastGpsPos_ = zero;
    lastGpsTime_ = ros::Time::now();
}

ImuOdom::~ImuOdom() {}

void ImuOdom::ImuCallback(const sensor_msgs::Imu& msg)
{
    if (firstT_)
    {
        time_ = msg.header.stamp;
        deltaT_ = 0;
        setGravity(msg.linear_acceleration);
        firstT_ = false;
    }
    else
    {
        deltaT_ = (msg.header.stamp - time_).toSec(); // 当前帧和上一帧的时间差
        time_ = msg.header.stamp;
        
        odom_.header.seq = msg.header.seq;
        odom_.header.stamp = msg.header.stamp;
        
        calcOrientation(msg.angular_velocity); // 计算角度，四元数表示
        calcPosition(msg.linear_acceleration); // 计算位置
        updateOdom(point_);
    }
}

void ImuOdom::GpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // GPS位置更新
    Eigen::Vector3d gpsPos(msg->pose.position.x, msg->pose.position.y, 0);

    // 重新初始化 IMU 系统
    point_.pos = gpsPos;  // 设定IMU的当前位置为GPS位置
    point_.v.setZero();    // 设定IMU的速度为零
    point_.orien = Eigen::Matrix3d::Identity();  // 设定IMU的姿态为单位矩阵（即没有旋转）

    // 使用IMU数据推算位置，并根据GPS位置进行插值
    point_.pos = interpolatePosition(time_, lastGpsTime_, point_.pos, gpsPos);
    
    lastGpsPos_ = gpsPos;
    lastGpsTime_ = ros::Time::now();
}


void ImuOdom::setGravity(const geometry_msgs::Vector3& msg)
{
    gravity_[0] = msg.x;
    gravity_[1] = msg.y;
    gravity_[2] = msg.z;
}

void ImuOdom::calcOrientation(const geometry_msgs::Vector3& msg)
{
    point_.w << msg.x, msg.y, msg.z;
    
    // 基于旋转矩阵表示方法
    Eigen::Matrix3d B;
    B << 0, -msg.z * deltaT_, msg.y * deltaT_, 
         msg.z * deltaT_, 0, -msg.x * deltaT_,
         -msg.y * deltaT_, msg.x * deltaT_, 0;

    double sigma = std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) * deltaT_;
    
    // 罗德里格斯公式
    point_.orien = point_.orien *
                    (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B -
                     ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
}

void ImuOdom::calcPosition(const geometry_msgs::Vector3& msg)
{
    Eigen::Vector3d acc_l(msg.x, msg.y, msg.z); // imu坐标系下的加速度
    Eigen::Vector3d acc_g = point_.orien * acc_l; // 转化到里程计坐标系下的加速度
    point_.v = point_.v + deltaT_ * (acc_g - gravity_); // 积分得到速度
    point_.pos = point_.pos + deltaT_ * point_.v; // 积分得到位置
}

void ImuOdom::updateOdom(const Point& point)
{
    // 位置
    odom_.pose.pose.position.x = point.pos(0);
    odom_.pose.pose.position.y = point.pos(1);
    odom_.pose.pose.position.z = point.pos(2);

    ROS_INFO("%f,%f,%f\r\n",odom_.pose.pose.position.x,odom_.pose.pose.position.y,odom_.pose.pose.position.z);

    // 姿态 四元数
    // odom_.pose.pose.orientation.x = (point.orien(2,1) - point.orien(1,2)) / 4;
    // odom_.pose.pose.orientation.y = (point.orien(0,2) - point.orien(2,0)) / 4;
    // odom_.pose.pose.orientation.z = (point.orien(1,0) - point.orien(0,1)) / 4;
    // odom_.pose.pose.orientation.w = std::sqrt(1 + point.orien(0,0) + point.orien(1,1) + point.orien(2,2)) / 2;
    Eigen::Quaterniond quat(point.orien);
    odom_.pose.pose.orientation.x = quat.x();
    odom_.pose.pose.orientation.y = quat.y();
    odom_.pose.pose.orientation.z = quat.z();
    odom_.pose.pose.orientation.w = quat.w();

    // 线速度
    odom_.twist.twist.linear.x = point.v(0);
    odom_.twist.twist.linear.y = point.v(1);
    odom_.twist.twist.linear.z = point.v(2);

    // 角速度
    odom_.twist.twist.angular.x = point.w(0);
    odom_.twist.twist.angular.y = point.w(1);
    odom_.twist.twist.angular.z = point.w(2);

    // 发布里程计
    odompub_.publish(odom_);
}

Eigen::Vector3d ImuOdom::interpolatePosition(const ros::Time& t_imu, const ros::Time& t_gps,
                                              const Eigen::Vector3d& p_imu, const Eigen::Vector3d& p_gps)
{
    // 线性插值
    double alpha = (t_imu - t_gps).toSec();
    Eigen::Vector3d p_interp = p_imu + alpha * (p_gps - p_imu);
    return p_interp;
}

#endif
