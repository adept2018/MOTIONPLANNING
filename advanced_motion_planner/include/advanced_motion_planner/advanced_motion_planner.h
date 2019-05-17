#ifndef ADVANCED_MOTION_PLANNER_H
#define ADVANCED_MOTION_PLANNER_H
/** Advanced Motion Planner (AMP) source file
  * Originally created from Basic Motion Planner (BMP)

  * History:
  * 2019-03-20  Changed from BMP by Alexander Konovalenko
  * 2019-04-23  Successfully tested on the car. Lightning in the room
  *             can negatively affect the LIDAR!!!
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
