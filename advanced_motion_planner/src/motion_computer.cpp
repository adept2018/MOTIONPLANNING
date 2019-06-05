/** Advanced Motion Planner (AMP) source file
  * Originally created from Basic Motion Planner (BMP)

  * History:
  * 2019-03-20  Changed from BMP by Alexander Konovalenko
  * 2019-04-23  Successfully tested on the car. Lightning in the room
  *             can negatively affect the LIDAR!!!
  * 2019-05-05  Some performance improvements are reversed back in calcLargestRectangularDirection
  *
  **/

#include <advanced_motion_planner/motion_computer.h>
#include <advanced_motion_planner/amp_common.h>

MotionComputer::MotionComputer(ros::NodeHandle &nh) {
    scan_sub = nh.subscribe("/scan", 10, &MotionComputer::scanCallBack, this);
}

void MotionComputer::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
    laser_scan = *scan;
    scan_queue.push(laser_scan);
}

bool MotionComputer::computeMotion() {
    float theta = 0.0f, r = 1.0f;

    while (!scan_queue.empty()) {
        sensor_msgs::LaserScan scan = scan_queue.front();
        scan_queue.pop();

        direction.clear();
        visibleCloud.clear();
        invisibleCloud.clear();
        #ifdef FUNCTIONAL_DEBUG_INFO
          pathCloud.clear();
        #endif

        laserScanToPointCloud.scanToCloud(visibleCloud, invisibleCloud, scan);

        int numberOfPoints = visibleCloud.size();

        if (numberOfPoints > 0) {

            //-------------------------------------------------
            // METHOD-1 average weighted angle based decision:
            //-------------------------------------------------
            //theta = getWeightedAverageDirection(numberOfPoints);

            //-------------------------------------------------
            // METHOD-2 analyze what is the lagest rectangular path available at the front
            //-------------------------------------------------
            calcLargestRectangularDirection(numberOfPoints, laserScanToPointCloud);
            r = RAW.x;
            theta = RAW.y;


            // some debug output:
            #ifdef DEBUG1
              std::cout << "AMP theta (deg)\t" << RAD2DEG(theta) << std::endl;
            #endif

            direction.push_back(r * cosf(theta));
            direction.push_back(r * sinf(theta));
            direction.push_back(theta);
        }
    }
    return true;
}

/**
  * This is improved version of the original BMP.
  * The weighted average angle is:
  *   <agnle> = sum(dist_i * angle_i) / sum(dist_i)
  * The forward motion (if free) is prioritized.
  */
float MotionComputer::getWeightedAverageDirection(const int n) {
  float sum_theta = 0.0f, tmp, sum_r = 0.0f, r, theta_w = 0.0f;
  bool free_front = true;

  if(n > 0) {
    // there are data to check
    for (int i = 0; i < n; i++) {
        float x = visibleCloud.points[i].x;
        float y = visibleCloud.points[i].y;
        AMP_utils::xy2polar(r, tmp, x, y);
        sum_theta += tmp * r;
        sum_r += r;
        // determine if there is an obsticle at the front:
        if(fabs(tmp) < RmaxHalfWidthAngle)
           free_front = false;
    }

    // Dived angle by number of points, i.e. average angle in original BMP
    //float theta_w = sum_theta / n;
    theta_w = sum_theta / sum_r;

    // Decide which way to turn away from obstacle
    if(!free_front) {
        theta_w += (theta_w < 0.0f) ? turn_offset : -turn_offset;
    } else {
        theta_w = 0.0f;  // move forward
    }
  }

  return theta_w;
}

void MotionComputer::calcLargestRectangularDirection(const int n, const LaserScanToPointCloud &ls) {
  register float a_best = 0.0f, r_best = 0.0f, w_best = 0.0f, a_i, r_i, w_i;
  uint counter = 0;

  if(n > 0) {
    // there are data to check

    initBestPathsCacher();

    // loop through path widths:
    for(w_i = minPathWidth; w_i <= maxPathWidth; w_i += pathWidthPitch) {

      float amin = -angle_range + pathsAngPitchHalf;
      float amax = angle_range - pathsAngPitchHalf;
      // optimize for performance:
      /*float amin = ls.statVis.Aminmax.x + pathsAngPitchHalf;
      float amax = ls.statVis.Aminmax.y - pathsAngPitchHalf; */
      // loop through all paths (angles):
      for(a_i = amin; a_i <= amax; a_i += pathsAngPitch) {

        // loop: increment distance (far end):
        for(minPathRange; r_i < MAX_FRONT_Range; r_i += pathsDistPitch) {
        //for(r_i = (MIN_FRONT_Range + pathsDistPitch); r_i < ls.statVis.Rminmax.y; r_i += pathsDistPitch) {

          // debug:
          #ifdef DEBUG2
            pcl::PointXY tmpp = AMP_utils::polar2PointXY(r_i, a_i);
            std::cout << "xyra_i:\t" << tmpp.x << "\t" << tmpp.y << "\t" << \
            r_i << "\t" << RAD2DEG(a_i) << "\t" << std::endl;
          #endif

          ++counter;

          //if(!areAnyPointsInsideRectangle(n, r_i, a_i, w_i)) {
          if(!tooManyPointsInsideRectangle(n, r_i, a_i, w_i)) {
            // if no points belong to then memorize that direction and max dist
            // we choose direction by the longest/widest/strightest available path
            bool longest = (r_i >= r_best);
            bool widest = (w_i >= w_best);
            bool strightest = (a_i <= a_best);
            bool maxarea = ((r_i*w_i) >= (r_best*w_best));
            //if(longest && widest && strightest) {
            if(maxarea && strightest) {
              // ideal path
              r_best = r_i; a_best = a_i; w_best = w_i;
              #ifdef DEBUG1
                std::cout << counter << "th new best r&w&a:\t" << r_best << "\t" << w << "\t" << RAD2DEG(a_best) << std::endl;
              #endif
            } else if(longest && widest ) {
              // less ideal path, we need to turn
              r_best = r_i; a_best = a_i; w_best = w_i;
              #ifdef DEBUG1
                std::cout << counter << "th new best r&w&a:\t" << r_best << "\t" << w << "\t" << RAD2DEG(a_best) << std::endl;
              #endif
            } else {
              // the worse case
            }
          } else {
            // there are points inside this path,
            // skip r_i+1 increment continue to another direction a_i+1
            break; // from r_i loop
          }

        } // loop on distance
        updateBestPathsCacher(r_best, a_best, w_best);
      } // loop on angle (direction)
    } // loop on path width

    findStraightestPathFromPathsCacher(r_best, a_best, w_best);

    #ifdef FUNCTIONAL_DEBUG_INFO
      // note that actual steering angle can be overwritten in advanced_motion_planner.cpp
      buildPathCloud(r_best, a_best, w_best);
    #endif
  }

  // return the best direction parameters
  RAW.x = r_best;
  RAW.y = a_best;
  RAW.z = w_best;
}

inline void  MotionComputer::initBestPathsCacher() {
  for(register int j = 0; j < BEST_PATHS_LEN; ++j) {
    bestPathsCacherRAW[0][j] = 0.0f;
    bestPathsCacherRAW[1][j] = PI;
    bestPathsCacherRAW[2][j] = 0.0f;
  }
  bestPathsCacherCounter = 0;
}

inline void MotionComputer::updateBestPathsCacher(const float r, const float a, const float w) {
  uint idx = bestPathsCacherCounter % BEST_PATHS_LEN;
  if(idx < BEST_PATHS_LEN) {
    bestPathsCacherRAW[0][idx] = r;
    bestPathsCacherRAW[1][idx] = a;
    bestPathsCacherRAW[2][idx] = w;
    ++bestPathsCacherCounter;
  } else {
    // OOPS! something is very wrong :-o
    std::cout << "WARNING in AMP updateBestPathsCacher: reseting bestPathsCacherCounter from " \
      << bestPathsCacherCounter << std::endl;
    bestPathsCacherCounter = 0U;
  }
}

// if not found then false is returned
// this function overrides inputs!
inline bool MotionComputer::findStraightestPathFromPathsCacher(float &r, float &a, float &w) {
  bool res = false;
  register float best_raw[3] = {r, a, w};

  for(int i = 0; i < BEST_PATHS_LEN; ++i) {
    if(fabsf(bestPathsCacherRAW[1][i]) < fabsf(best_raw[1])) {
      best_raw[0] = bestPathsCacherRAW[0][i];
      best_raw[1] = bestPathsCacherRAW[1][i];
      best_raw[2] = bestPathsCacherRAW[2][i];
    }
    #ifdef DEBUG1
      std::cout << "BestPaths (ra)\t" << i << "th\t" << bestPathsCacherRAW[0][i] << "\t" << \
        RAD2DEG(bestPathsCacherRAW[1][i]) << "\t" << bestPathsCacherRAW[2][i] << std::endl;
    #endif
  }
  if(best_raw[0] == 0.0f || best_raw[1] == PI) {
    // something is wrong, no modifications to r & a
  } else {
    // Straighter path found
    #ifdef DEBUG1
      std::cout << "BestSelected (raw)\t" << best_raw[0] << "\t" <<  RAD2DEG(best_raw[1]) << "\t" << best_raw[2] << std::endl;
      std::cout << "Instead of a (raw)\t" << r << "\t" <<  RAD2DEG(a) << "\t" << w << std::endl;
    #endif
    r = best_raw[0];
    a = best_raw[1];
    w = best_raw[2];
    res = true;
  }

  return res;
}

bool MotionComputer::areAnyPointsInsideRectangle(const int n, const float r_i, const float a_i, const float w_i) {
  bool isAnyInside = false;
  pcl::PointXY p, A, B, C, D;

  // construct 4 rectangle corners
  buildRectangle(A, B, C, D, r_i, a_i, w_i);

  // loop through all visible points
  for (int i = 0; i < n; i++) {
      p.x = visibleCloud.points[i].x;
      p.y = visibleCloud.points[i].y;
      // determine if no any points reside inside the rectangle
      if(AMP_utils::isInsideRectangle(p, A, B, C, D)) {
        isAnyInside = true;
        #ifdef DEBUG2
          // print point inside the reactangle
          float rr, aa;
          AMP_utils::PointXY2polar(rr, aa, p);
          std::cout << "Pnt xyra:\t" << p.x << "\t" << p.y << "\t" << \
          rr << "\t" << RAD2DEG(aa) << std::endl;
        #endif
        break;
      } else {
        // no points inside!!!
        // this is for debug or statistics only
      }
  }

  return isAnyInside;
}

bool MotionComputer::tooManyPointsInsideRectangle(const int n, const float r_i, const float a_i, const float w_i) {
  bool tooManyInside = false;
  int pointsFound = countPointsInsideRectangle(n, r_i, a_i, w_i);

  tooManyInside = (pointsFound <= MAX_ALLOWED_POINTS) ? false : true;

  return tooManyInside;
}

int MotionComputer::countPointsInsideRectangle(const int n, const float r_i, const float a_i, const float w_i) {
  int countedInside = 0;
  pcl::PointXY p, A, B, C, D;

  // construct 4 rectangle corners
  buildRectangle(A, B, C, D, r_i, a_i, w_i);

  // loop through all visible points
  for (int i = 0; i < n; i++) {
      p.x = visibleCloud.points[i].x;
      p.y = visibleCloud.points[i].y;
      // determine if no any points reside inside the rectangle
      if(AMP_utils::isInsideRectangle(p, A, B, C, D)) {
        ++countedInside;
        #ifdef DEBUG2
          // print point inside the reactangle
          float rr, aa;
          AMP_utils::PointXY2polar(rr, aa, p);
          std::cout << "Pnt xyra:\t" << p.x << "\t" << p.y << "\t" << \
          rr << "\t" << RAD2DEG(aa) << std::endl;
        #endif
        //break;
      } else {
        // no points inside!!!
        // this is for debug or statistics only
      }
  }

  return countedInside;
}

// build rectangle for the possible motion path in the direction of polar angle a
// Width is carWidth_m, the length is (MIN_FRONT_Range + r)
//
//  ^
//  |
// Y|    A   /
//  |       /   B
//  |      /
//  | D   /
//  |    /   C
//  |
//  |car is
//  |here
//  +-------------->
//                X
//
void MotionComputer::buildRectangle(pcl::PointXY &A, pcl::PointXY &B, pcl::PointXY &C, pcl::PointXY &D, const float r, const float a, const float w) {
    register float Ar, Aa, Br, Ba, Cr, Ca, Dr, Da, tmp_cw2, tmp_far_a;

    tmp_cw2 = w * w * 0.25f;

    // building polar coordinates
    Ar = sqrtf( tmp_cw2 + r * r);
    tmp_far_a = asinf(w * 0.5f / Ar);
    Aa = a - tmp_far_a;
    Ba = a + tmp_far_a;
    Br = Ar;
    Ca = a + RminHalfWidthAngle;
    Cr = sqrtf(tmp_cw2 + minPathRange * minPathRange);
    Da = a - RminHalfWidthAngle;
    Dr = Cr;

    // converting to XY
    AMP_utils::polar2xy(A.x, A.y, Ar, Aa);
    AMP_utils::polar2xy(B.x, B.y, Br, Ba);
    AMP_utils::polar2xy(C.x, C.y, Cr, Ca);
    AMP_utils::polar2xy(D.x, D.y, Dr, Da);

    // DEBUG:
    #ifdef DEBUG2
      std::cout << "A\t" << A.x << "\t"<< A.y << "\t" << Ar << "\t" << RAD2DEG(Aa) << std::endl;
      std::cout << "B\t" << B.x << "\t"<< B.y << "\t" << Br << "\t" << RAD2DEG(Ba) << std::endl;
      std::cout << "C\t" << C.x << "\t"<< C.y << "\t" << Cr << "\t" << RAD2DEG(Ca) << std::endl;
      std::cout << "D\t" << D.x << "\t"<< D.y << "\t" << Dr << "\t" << RAD2DEG(Da) << std::endl;
    #endif
}

#ifdef FUNCTIONAL_DEBUG_INFO
// this is for vizualisation of the planned path in rviz, not useful or used for car
void MotionComputer::buildPathCloud(const float r, const float a, const float w) {
  pcl::PointXY A, B, C, D;

  // buiding the corners (although data could be reused from the last call, too lazy to implement...)
  buildRectangle(A, B, C, D, r, a, w);

  // the rectangle points are send here to the cloud for publishing (visualizaion for rviz only)
  // points along each side  are produced from 4 points A..D
  for(uint i = 0; i < maxSidePointsInPathCloud; ++i) {
     pathCloud.push_back(AMP_utils::getPointInBetween(A, B, i, maxSidePointsInPathCloud));
     pathCloud.push_back(AMP_utils::getPointInBetween(B, C, i, maxSidePointsInPathCloud));
     pathCloud.push_back(AMP_utils::getPointInBetween(C, D, i, maxSidePointsInPathCloud));
     pathCloud.push_back(AMP_utils::getPointInBetween(D, A, i, maxSidePointsInPathCloud));
  }

  #ifdef DEBUG2
    pcl::PointXY p = AMP_utils::polar2PointXY(r, a);
    std::cout << "{raxy},ABCD_xy\t" << r << "\t"<< RAD2DEG(a) << "\t" << p.x << "\t"<< p.y << "\t";
    std::cout << A.x << "\t"<< A.y << "\t" << B.x << "\t" << B.y << "\t";
    std::cout << C.x << "\t"<< C.y << "\t" << D.x << "\t" << D.y << std::endl;
  #endif
}
#endif
