#include "pose_subscriber.h"  // 引入头文件
#include "utils.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>


static PositionStructure Position;
static ScanPositionStructure ScanPosition;

// 订阅话题回调函数
static void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // 倾斜补偿
    setPosition(&Position,msg->pose.position.x, msg->pose.position.y);
}

static void setPosition(PositionStructure *pos,float x,float y)
{
    static float l_x,l_y;

    float x_d_temp = x;
    float y_d_temp = y;

    pos->x = KalmanFilter_Update(&xDisKalmanFilter,x_d_temp);
    pos->y = KalmanFilter_Update(&yDisKalmanFilter,y_d_temp);

    float x_s_temp = (pos->x-l_x)/0.1f;
    float y_s_temp = (pos->y-l_y)/0.1f;

    pos->x_s = applyLowPassFilter(&x_sLowPassFilter,x_s_temp);
    pos->y_s = applyLowPassFilter(&y_sLowPassFilter,y_s_temp);

    // ROS_INFO("%f,%f\r\n",pos->y*100.0,pos->y_s*100.0);

    l_x = pos->x;
    l_y = pos->y;
}
PositionStructure getPosition(void)
{
    return Position;
}

// 订阅话题的函数
ros::Subscriber subscribeToPoseTopic(ros::NodeHandle& nh)
{
    return nh.subscribe("slam_out_pose", 1000, poseCallback);
}






static void setScanPosition(ScanPositionStructure *pos,float x,float y)
{
    static float l_x,l_y;

    float x_d_temp = x;
    float y_d_temp = y;

    pos->x = KalmanFilter_Update(&xScanDisKalmanFilter,x_d_temp);
    pos->y = KalmanFilter_Update(&yScanDisKalmanFilter,y_d_temp);

    float x_s_temp = (pos->x-l_x)/0.1f;
    float y_s_temp = (pos->y-l_y)/0.1f;

    pos->x_s = applyLowPassFilter(&x_sScanLowPassFilter,x_s_temp);
    pos->y_s = applyLowPassFilter(&y_sScanLowPassFilter,y_s_temp);

    // ROS_INFO("%f,%f",pos->y*100.0,pos->y_s*100.0);

    l_x = pos->x;
    l_y = pos->y;
}
ScanPositionStructure getScanPosition(void)
{
    return ScanPosition;
}
void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    float desired_angle_deg = -90.0;
    float desired_angle_rad = desired_angle_deg * M_PI / 180.0;

    // 检查目标角度是否在扫描范围内
    if (desired_angle_rad < msg->angle_min || desired_angle_rad > msg->angle_max)
    {
        ROS_WARN("Desired angle %.2f° is out of scan range (%.2f° to %.2f°)",\
                 desired_angle_deg, msg->angle_min * 180.0 / M_PI, msg->angle_max * 180.0 / M_PI);
        return;
    }

    // 计算目标角度对应的索引
    int index = round((desired_angle_rad - msg->angle_min) / msg->angle_increment);
    static float distance_temp = 0;

    float distance = std::numeric_limits<float>::infinity();
    int search_limit = 5;  // 最多向两边查找±5个点
    int found_index = -1;

    // 在 index 附近寻找最近的有效点
    for (int offset = 0; offset <= search_limit; ++offset)
    {
        for (int sign = -1; sign <= 1; sign += 2) // sign=-1（左），sign=1（右）
        {
            int search_index = index + sign * offset;

            if (search_index >= 0 && search_index < msg->ranges.size())
            {
                float d = msg->ranges[search_index];
                if (std::isfinite(d) && d > 0.0)
                {
                    distance = d;
                    found_index = search_index;
                    break;
                }
            }
        }
        if (found_index != -1)
            break;
    }

    if (found_index != -1)
    {
        float actual_angle = msg->angle_min + found_index * msg->angle_increment;
        // ROS_INFO("Valid point found near %.2f° at %.2f°, distance: %.2f meters",\
                 desired_angle_deg, actual_angle * 180.0 / M_PI, distance);
        distance_temp = distance;
        setScanPosition(&ScanPosition, 0, distance);
    }
    else
    {
        // ROS_WARN("No valid scan data found near %.2f°", desired_angle_deg);\
        setScanPosition(&ScanPosition, 0, distance_temp); // 使用上一次有效数据
    }
}





ros::Subscriber subscribeToScanPoseTopic(ros::NodeHandle& nh)
{
    return nh.subscribe("scan", 1000, ScanCallback);
}

