#ifndef AMP_COMMON_H
#define AMP_COMMON_H
/** Advanced Motion Planner (AMP) source file
  * Originally created from Basic Motion Planner (BMP)

  * History:
  * 2019-03-20  Created by Alexander Konovalenko
  * 2019-04-23  Successfully tested on the car. Lightning in the room
  *             can negatively affect the LIDAR!!!
  *
  **/

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
/** Main configuration parameters and constants
  * used in the AMP module
  */

// print debug info on with different detail level 1, 2 ...
//#define DEBUG1
//#define DEBUG2
// publish and/or print extended data/clouds/...
#define FUNCTIONAL_DEBUG_INFO
#define OFFCAR_DEBUG

#ifdef FUNCTIONAL_DEBUG_INFO
  #define maxSidePointsInPathCloud 10U
#endif

#define BACKWARD_MOTION

// naming constants for the publisher and data/clouds
const std::string AMP_NAME = "amp";
const std::string CLOUD_VIS_NAME = AMP_NAME + "/cloud/visible";
const std::string CLOUD_INVIS_NAME = AMP_NAME + "/cloud/invisible";
const std::string POSE_VECTOR = AMP_NAME + "/pose";
const std::string NORMDIR_VECTOR = AMP_NAME + "/derection";
const std::string VESC_NAME = "vesc/high_level/ackermann_cmd_mux/input/default";
#ifdef FUNCTIONAL_DEBUG_INFO
  const std::string PATH_CLOUD = AMP_NAME + "/cloud/pathCloud";
#endif
#ifdef OFFCAR_DEBUG
  const std::string OFFCAR = "_DBG";
#else
  const std::string OFFCAR = "";
#endif

#ifndef PI
  #define   PI                3.141592654f
#endif
#ifndef   DEG2RAD
  #define DEG2RAD(x)          (x * PI / 180.0f)
#endif
#ifndef   RAD2DEG
  #define RAD2DEG(x)          (x * 180.0f / PI)
#endif

#define   LIDAR_ANG_OFFSET    DEG2RAD(90.0f)

// Offset to turn away from obstacle (old BMP)
#define   turn_offset         DEG2RAD(28.6f)

// Limiting angle to turn wheels:
//#define   SteeringAngleLimit  0.34f // default 19.4deg in BMP:
//#define   SteeringAngleLimit  DEG2RAD(19.4f)  // defualt in BMP
#define   SteeringAngleLimit  DEG2RAD(29.4f)

// LIDAR data filtering (for visibleCloud) withing following intervals:
#define   MAX_FRONT_Range     4.5f
//#define   MIN_FRONT_Range     0.10f // tested OK on 2019-04-23
#define   MIN_FRONT_Range     0.15f
//#define   angle_range         DEG2RAD(28.6f)  // default in BMP
//#define   angle_range         DEG2RAD(90.0f)    // tested in car 2019-04-23
#define   angle_range         DEG2RAD(70.0f)
// filtering parameters for moving back (back off):
#define   MAX_BACK_Range      0.70f
#define   MIN_BACK_Range      0.30f
#define   angle_range_Back    DEG2RAD(45.0f)
#define   MAX_ALLOWED_POINTS  2 // this is filtering of the noisy points

// safe width & corresponding half angle width where car can go through within MAX_FRONT_Range
#define   carWidth_m          0.33f
#define   RmaxHalfWidthAngle  atan2f(carWidth_m / 2.0f, MAX_FRONT_Range)
#define   RminHalfWidthAngle  atan2f(carWidth_m / 2.0f, MIN_FRONT_Range)

// rectangular path determination algorithm relevant
#define   pathsAngPitch       (2.0f * RmaxHalfWidthAngle)
#define   pathsAngPitchHalf   (0.5f * pathsAngPitch)
#define   pathsNumber         (2.0f * angle_range / pathsAngPitch)
#define   pathsDistPitch      0.3f   // in m, increments in r for the rectangle path finding
#define   minPathWidth        (carWidth_m * 1.2f)
#define   maxPathWidth        (minPathWidth * 5.0f)
#define   minPathRange        (MIN_FRONT_Range * 0.5f)
#define   pathWidthPitch      ((maxPathWidth - minPathWidth) / 4.0)
#define   AREA_TOLERANCE      1.0e-5f
#define   BEST_PATHS_LEN      9     // a number of paths to cache
#define   NO_GO_MIN_DIST      MIN_FRONT_Range  // in m

// misc parameters:
// seem that VESC cannot properly manage driving att speeds lower than 0.35 m/s
#define DRIVE_SPEED_DEFAULT   0.35f   // m/s
#ifdef BACKWARD_MOTION
  #define BACK_SPEED_DEFAULT -0.35f  // m/s
  #define BACK_DISTANCE       0.7f  // m
#endif

// This is calls with service static functions:
class AMP_utils {
  public:
    AMP_utils() {}

    // coordinate transformation functions:
    static void polar2xy(float &x, float &y, const float r, const float a);
    static void polar2xy(float* x, float* y, const float r, const float a);
    static void xy2polar(float &r, float &a, const float x, const float y);
    static pcl::PointXY polar2PointXY(const float r, const float a);
    static pcl::PointXYZ polar2PointXYZ(const float r, const float a);
    static void polar2PointXYZ(pcl::PointXYZ &p, const float r, const float a);
    static void PointXY2polar(float &r, float &a, const pcl::PointXY &p);
    static void PointXYZ2polar(float &r, float &a, const pcl::PointXYZ &p);

    // geometry related functions:
    static float area(const pcl::PointXY &A, const pcl::PointXY &B, const pcl::PointXY &C);
    static float area(const pcl::PointXY &A, const pcl::PointXY &B,
      const pcl::PointXY &C, const pcl::PointXY &D);
    static bool isInsideRectangle(const pcl::PointXY &p, const pcl::PointXY &A,
      const pcl::PointXY &B, const pcl::PointXY &C, const pcl::PointXY &D);

    // additional functions
    #ifdef FUNCTIONAL_DEBUG_INFO
      static pcl::PointXYZ getPointInBetween(const pcl::PointXY &A, const pcl::PointXY &B, const uint i, const uint max);
    #endif
};

class AMP_stat {
  public:
    pcl::PointXY Rminmax, Aminmax, Xminmax, Yminmax;
    bool isUpdated;

    AMP_stat() {isUpdated = false;}
    void resetStat();
    void updateStat(const float r, const float a, const pcl::PointXYZ &point);
    bool isStatInitialized();

};

#endif //AMP_COMMON_H
