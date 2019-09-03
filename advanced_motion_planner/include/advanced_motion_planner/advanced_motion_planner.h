#ifndef ADVANCED_MOTION_PLANNER_H
#define ADVANCED_MOTION_PLANNER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <advanced_motion_planner/motion_computer.h>
#include <advanced_motion_planner/package_structs.h>

// Publishers:
ros::Publisher m_pubAck;
ros::Publisher m_pubPose;
ros::Publisher m_pubDirection;
ros::Publisher m_pubMinObservationDirection;
ros::Publisher m_pubMaxObservationDirection;
ros::Publisher m_pubMap;
ros::Publisher m_pubObservedPoints;
ros::Publisher m_pubCarrot;
ros::Publisher m_pubWallOutline;

// Standard header frame id:
std::string HEADER_FRAME_ID = "amp";

#endif //ADVANCED_MOTION_PLANNER_H
