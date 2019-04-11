#ifndef ADVANCED_MOTION_PLANNER_H
#define ADVANCED_MOTION_PLANNER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <advanced_motion_planner/motion_computer.h>
#include <advanced_motion_planner/amp_common.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// Publishers:
ros::Publisher pubCloudVisible;
ros::Publisher pubCloudInvisible;
#ifdef FUNCTIONAL_DEBUG_INFO
ros::Publisher pubPathCloud;
#endif
ros::Publisher pubPose;
ros::Publisher pubDirection;
ros::Publisher pubAck;

#endif //ADVANCED_MOTION_PLANNER_H
