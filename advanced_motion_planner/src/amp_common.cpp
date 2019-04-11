#include <advanced_motion_planner/amp_common.h>
#include <cmath>

// the following are service functions

void AMP_utils::polar2xy(float &x, float &y, const float r, const float a) {
  x = r * cosf(a);
  y = r * sinf(a);
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
    //res.y = fminf(A.y, B.y) + fabsf(B.y - A.y) * ii;
    res.y = a * res.x + b;
    res.z = 0.0f;

    return res;
  }

#endif
