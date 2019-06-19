#ifndef ADVANCED_MOTION_PLANNER_H
#define ADVANCED_MOTION_PLANNER_H
/** Advanced Motion Planner (AMP) source file
  * Originally created from Basic Motion Planner (BMP)

  * History:
  * 2019-03-20  Changed from BMP by Alexander Konovalenko
  * 2019-04-23  Successfully tested on the car. Lightning in the room
  *             can negatively affect the LIDAR!!!
  * 2019-06-19  Demo has been performed on 190618. This is final commit to AMP.
  *            Things left for further improvements:
  *         1) extend decision making in motion_computer.cpp lines# 128-132,
  *          as Chrais pointed out, ratio r_best/w_best can be used with proper
  *          range (e.g. 0.8...1.2) check as additional condition on selection
  *          of the best path in lines# 148-170 of motion_computer.cpp
  *         2) Alan's implementation of reading parameters from a file and
  *          their real-time update shall be used instead of many macro defines
  *          in amp_common.h
  *         3) Implemented backward motion shall be further debugged and improved
  *          (it is disabled right now in amp_common.h).
  *         4) Positive noise (appearing ghost points) filtering is disabled
  *          (macro MAX_ALLOWED_POINTS) because of bigger problem with missing
  *          points (negative noise).
  *         5) BETTER LIDAR IS WANTED!!! 
  *
  **/

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
ros::Publisher pubPoseVector;
ros::Publisher pubNormDirection;
ros::Publisher pubAck;

#endif //ADVANCED_MOTION_PLANNER_H
