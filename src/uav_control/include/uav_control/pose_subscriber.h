#ifndef POSE_SUBSCRIBER_H
#define POSE_SUBSCRIBER_H

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

typedef struct
{
    float x;
    float y;
    float x_s;
    float y_s;
    float Angle;
    float Rad;
} PositionStructure;

// 订阅话题的函数
ros::Subscriber subscribeToPoseTopic(ros::NodeHandle& nh);

static void setPosition(PositionStructure *pos,float x,float y,float a);
PositionStructure getPosition(void);




typedef struct
{
    float x;
    float y;
    float x_s;
    float y_s;
}ScanPositionStructure;

static void setScanPosition(ScanPositionStructure *pos,float x,float y);
ros::Subscriber subscribeToScanPoseTopic(ros::NodeHandle& nh);
ScanPositionStructure getScanPosition(void);

#endif // POSE_SUBSCRIBER_H
