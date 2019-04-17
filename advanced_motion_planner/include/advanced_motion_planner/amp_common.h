#ifndef AMP_COMMON_H
#define AMP_COMMON_H
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
/** Main configuration parameters and constants
  * used in the AMP module
  */

// print debug info on with different detail level 1, 2 ...
//#define DEBUG1
//#define DEBUG2
// publish and/or print extended data/clouds/...
//#define FUNCTIONAL_DEBUG_INFO

#ifdef FUNCTIONAL_DEBUG_INFO
  #define maxSidePointsInPathCloud 10U
#endif

#ifndef PI
  #define   PI                  3.141592654f
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
#define   SteeringAngleLimit  DEG2RAD(19.4f)

// LIDAR data filtering (for visibleCloud) withing following intervals:
#define   max_range           2.0f
#define   min_range           0.10f
//#define   angle_range         DEG2RAD(28.6f)  // default in BMP
#define   angle_range         DEG2RAD(45.0f)

// safe width & corresponding half angle width where car can go through within max_range
#define   carWidth_m          0.3f
#define   RmaxHalfWidthAngle  atan2f(carWidth_m / 2.0f, max_range)
#define   RminHalfWidthAngle  atan2f(carWidth_m / 2.0f, min_range)

// rectangular path determination algorithm relevant
#define   pathsAngPitch       (2.0f * RmaxHalfWidthAngle)
#define   pathsAngPitchHalf   (0.5f * pathsAngPitch)
#define   pathsNumber         (2.0f * angle_range / pathsAngPitch)
#define   pathsDistPitch      0.3f   // in m, increments in r for the rectangle path finding
#define   AREA_TOLERANCE      1.0e-4f

#define DRIVE_SPEED_DEFAULT   0.35f  // m/s


// This is calls with service static functions:
class AMP_utils {
  public:
    AMP_utils() {}

    // coordinate transformation functions:
    static void polar2xy(float &x, float &y, const float r, const float a);
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

#endif //AMP_COMMON_H
