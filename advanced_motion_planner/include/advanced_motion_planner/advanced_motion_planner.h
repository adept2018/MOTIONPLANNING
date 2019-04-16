#ifndef ADVANCED_MOTION_PLANNER_H
#define ADVANCED_MOTION_PLANNER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <advanced_motion_planner/motion_computer.h>

// Publishers:
ros::Publisher m_pubCloud;
ros::Publisher m_pubPose;
ros::Publisher m_pubDirection;
ros::Publisher m_pubAck;

#endif //ADVANCED_MOTION_PLANNER_H
