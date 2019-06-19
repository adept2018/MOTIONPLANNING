#ifndef MOTION_COMPUTER_H
#define MOTION_COMPUTER_H
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
#include <cmath>
#include <queue>
#include <sensor_msgs/LaserScan.h>
#include <advanced_motion_planner/laserscan_to_pointcloud.h>
#include <advanced_motion_planner/amp_common.h>


class MotionComputer {
private:
    LaserScanToPointCloud laserScanToPointCloud;
    sensor_msgs::LaserScan laser_scan;
    bool acquired_scan;
    std::queue<sensor_msgs::LaserScan> scan_queue;
    // Subscriber:
    ros::Subscriber scan_sub;
    // database of best paths (pairs of distanses R, angles A and widths W):
    float bestPathsCacherRAW[3][BEST_PATHS_LEN];
    uint bestPathsCacherCounter;

    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan);

public:
    std::vector<float> direction;
    // the following is the best R, Angle and Width.
    pcl::PointXYZ RAW;
    pcl::PointCloud<pcl::PointXYZ> visibleCloud;
    pcl::PointCloud<pcl::PointXYZ> invisibleCloud;
    #ifdef FUNCTIONAL_DEBUG_INFO
      pcl::PointCloud<pcl::PointXYZ> pathCloud;
    #endif

    MotionComputer(ros::NodeHandle &nh);
    bool computeMotion();
    // implemented methods for determination of motion direction:
    float getWeightedAverageDirection(const int NoOfPoints);
    bool calcLargestRectangularDirection(const int NoOfPoints, const LaserScanToPointCloud &ls);
    void buildRectangle(pcl::PointXY &A, pcl::PointXY &B, pcl::PointXY &C, pcl::PointXY &D,
      const float r, const float a, const float w);
    bool areAnyPointsInsideRectangle(const int n, const float r_i, const float a_i, const float w_i);
    bool tooManyPointsInsideRectangle(const int n, const float r_i, const float a_i, const float w_i);
    int countPointsInsideRectangle(const int n, const float r_i, const float a_i, const float w_i);
    void initBestPathsCacher();
    void updateBestPathsCacher(const float r, const float a, const float w);
    bool findStraightestPathFromPathsCacher(float &r, float &a, float &w);

    #ifdef FUNCTIONAL_DEBUG_INFO
      void buildPathCloud(const float r, const float a, const float w);
    #endif
};

#endif //MOTION_COMPUTER_H
