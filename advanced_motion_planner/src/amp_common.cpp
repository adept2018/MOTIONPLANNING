/** Advanced Motion Planner (AMP) source file
  * Originally created from Basic Motion Planner (BMP)

  * History:
  * 2019-03-20  Created by Alexander Konovalenko
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
#include <advanced_motion_planner/amp_common.h>
#include <cmath>

// the following are service functions

void AMP_utils::polar2xy(float &x, float &y, const float r, const float a) {
  x = r * cosf(a);
  y = r * sinf(a);
}

void AMP_utils::polar2xy(float* x, float* y, const float r, const float a) {
  float xx, yy;
  AMP_utils::polar2xy(xx, yy, r, a);
  *x = xx;
  *y = yy;
}

void AMP_utils::xy2polar(float &r, float &a, const float x, const float y) {
  r = sqrtf(x * x + y * y);
  a = atan2f(y, x);
}

// the return is polar pair of coordinates, r and angle.
pcl::PointXY AMP_utils::polar2PointXY(const float r, const float a) {
  pcl::PointXY p;
  AMP_utils::polar2xy(p.x, p.y, r, a);
  return p;
}

// the return is polar pair of coordinates, r and angle. Z is not modified
pcl::PointXYZ AMP_utils::polar2PointXYZ(const float r, const float a) {
  pcl::PointXYZ p;
  AMP_utils::polar2xy(p.x, p.y, r, a);
  return p;
}

void AMP_utils::polar2PointXYZ(pcl::PointXYZ &p, const float r, const float a) {
  AMP_utils::polar2xy(p.x, p.y, r, a);
}

void AMP_utils::PointXY2polar(float &r, float &a, const pcl::PointXY &p) {
  AMP_utils::xy2polar(r, a, p.x, p.y);
}

void AMP_utils::PointXYZ2polar(float &r, float &a, const pcl::PointXYZ &p) {
  AMP_utils::xy2polar(r, a, p.x, p.y);
}

// following 3 methods are taken from
// https://www.geeksforgeeks.org/check-whether-given-point-lies-inside-rectangle-not/
// area of the triangle ABC
float AMP_utils::area(const pcl::PointXY &A, const pcl::PointXY &B, const pcl::PointXY &C) {
  return fabs((A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y)) * 0.5f);
}

// area of the rectangular ABCD
float AMP_utils::area(const pcl::PointXY &A, const pcl::PointXY &B,
  const pcl::PointXY &C, const pcl::PointXY &D) {
  return AMP_utils::area(A, B, C) + AMP_utils::area(A, D, C);
}

bool AMP_utils::isInsideRectangle(const pcl::PointXY &P, const pcl::PointXY &A,
  const pcl::PointXY &B, const pcl::PointXY &C, const pcl::PointXY &D) {
  float area_ABCD = 0.0f, area_sum = 0.0f;

  // Calculate area of rectangle ABCD
  area_ABCD = AMP_utils::area(A, B, C, D);
  // Calculate area of triangle PAB
  area_sum = AMP_utils::area(P, A, B);
  // Calculate area of triangle PBC
  area_sum += AMP_utils::area(P, B, C);
  // Calculate area of triangle PCD
  area_sum += AMP_utils::area(P, C, D);
  // Calculate area of triangle PAD
  area_sum += AMP_utils::area(P, A, D);

  // Check the sum
  return ( fabs(area_ABCD - area_sum) < AREA_TOLERANCE );
}

// additional functions
#ifdef FUNCTIONAL_DEBUG_INFO

  pcl::PointXYZ AMP_utils::getPointInBetween(const pcl::PointXY &A, const pcl::PointXY &B, const uint i, const uint max) {
    pcl::PointXYZ res;
    float a = (B.y - A.y) / (B.x - A.x);
    float b = A.y - a * A.x;
    float ii = (i>=max) ? 1.0f : (float(i) / float(max));

    res.x = fminf(A.x, B.x) + fabsf(B.x - A.x) * ii;
    res.y = a * res.x + b;
    res.z = 0.0f;

    return res;
  }

#endif

float AMP_utils::distXY(const pcl::PointXY &A, const pcl::PointXY &B) {
  register float dx = A.x - B.x, dy = A.y - B.y;
  return sqrtf(dx * dx + dy * dy);
}

/*AMP_stat::AMP_stat() {
  resetStat();
}*/

void AMP_stat::updateStat(const float r, const float a, const pcl::PointXYZ &point) {
  if (Rminmax.x >= r) {
    Rminmax.x = r; //min r
    isUpdated = true;
  }
  if (Rminmax.y < r) {
    Rminmax.y = r; // max r
    isUpdated = true;
  }
  if (Aminmax.x >= a) {
    Aminmax.x = a; //min angle
    isUpdated = true;
  }
  if (Aminmax.y < a) {
    Aminmax.y = a; // max angle
    isUpdated = true;
  }
  if (Xminmax.x >= point.x) {
    Xminmax.x = point.x; //min x
    isUpdated = true;
  }
  if (Xminmax.y < point.x) {
    Xminmax.y = point.x; // max x
    isUpdated = true;
  }
  if (Yminmax.x >= point.y) {
    Yminmax.x = point.y; //min y
    isUpdated = true;
  }
  if (Yminmax.y < point.y) {
    Yminmax.y = point.y; // max y
    isUpdated = true;
  }
}

void AMP_stat::resetStat() {
  isUpdated = false;
  // the following default values are based on filtering of visibleCloud
  Rminmax.x = MAX_FRONT_Range;       // impossible value for min
  Rminmax.y = MIN_FRONT_Range;           // impossible value for max
  Aminmax.x = angle_range;          // impossible value for min
  Aminmax.y = -Aminmax.x; // impossible value for max
  Xminmax.x = MAX_FRONT_Range;       // impossible value for min
  Xminmax.y = -Xminmax.x; // impossible value for max
  Yminmax.x = Xminmax.x;  // impossible value for min
  Yminmax.y = -Yminmax.x;  // impossible value for max
}

// check if all ranges are simultaneously processed (i.e. min <= mmax)
bool AMP_stat::isStatInitialized() {
  bool res = false;
  if(Rminmax.x <= Rminmax.y) {
    if(Aminmax.x <= Aminmax.y) {
      if(Xminmax.x <= Xminmax.y) {
        if(Yminmax.x <= Yminmax.y) {
          res = true;
        }
      }
    }
  }
  return res;
}
