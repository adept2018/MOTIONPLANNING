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
    while (!scan_queue.empty()) {
        sensor_msgs::LaserScan scan = scan_queue.front();
        scan_queue.pop();

        direction.clear();
        visibleCloud.clear();
        invisibleCloud.clear();

        visibleCloud = laserScanToPointCloud.scanToCloud(scan, true);
        invisibleCloud = laserScanToPointCloud.scanToCloud(scan, false);

        int numberOfPoints = visibleCloud.size();

        if (numberOfPoints > 0) {

            // average weighted angle based decision:
            //float theta_w = getWeightedAverageDirection(numberOfPoints);

            // analyze what is the lagest rectangular path available at the front
            float theta_w = getLargestRectangularDirection(numberOfPoints);

            // some debug output:
            #ifdef DEBUG1
              std::cout << "AMP theta_w (deg)\t" << RAD2DEG(theta_w) << std::endl;
            #endif

            direction.push_back(cosf(theta_w));
            direction.push_back(sinf(theta_w));
            direction.push_back(theta_w);
        }
    }
    return true;
}

float MotionComputer::getWeightedAverageDirection(const int n) {
  float sum_theta = 0.0f, tmp, sum_r = 0.0f, r, theta_w = 0.0f;
  bool free_front = true;

  if(n > 0) {
    // there are data to check
    for (int i = 0; i < n; i++) {
        float x = visibleCloud.points[i].x;
        float y = visibleCloud.points[i].y;
        /*tmp = atan2f(y, x);
        r = sqrt(x * x + y * y);*/
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

float MotionComputer::getLargestRectangularDirection(const int n) {
  float a_best = 0.0f, r_best = 0.0f, a_i, r_i;
  unsigned int counter = 0;

  if(n > 0) {
    // there are data to check

    // loop through all paths (angles)
    for(a_i = -angle_range + pathsAngPitchHalf; a_i <= angle_range - pathsAngPitchHalf; a_i += pathsAngPitch) {
      // loop: increment distance (far end)
      for(r_i = (min_range + pathsDistPitch); r_i < max_range; r_i += pathsDistPitch) {

        // debug:
        #ifdef DEBUG2
          pcl::PointXY tmpp = AMP_utils::polar2PointXY(r_i, a_i);
          std::cout << "xyra_i:\t" << tmpp.x << "\t" << tmpp.y << "\t" << \
          r_i << "\t" << RAD2DEG(a_i) << "\t" << std::endl;
        #endif

        ++counter;

        if(!areAnyPointsInsideRectangle(n, r_i, a_i)) {
          // if no points belong to then memorize that direction and max dist
          // we choose direction by the longest available path
          if(r_i > r_best) {
            r_best = r_i;
            a_best = a_i;
            #ifdef DEBUG1
              std::cout << counter << "th new best r&a:\t" << r_best << "\t" << RAD2DEG(a_best) << std::endl;
            #endif
          }
        } else {
          // there are points inside this path, continue to another direction
          break; // from r_i loop
        }

      } // loop on distance
    } // loop on angle (direction)
  }

  // return the best direction
  return a_best;
}

bool MotionComputer::areAnyPointsInsideRectangle(const int n, const float r_i, const float a_i) {
  bool isAnyInside = false;
  pcl::PointXY p, A, B, C, D;

  // construct 4 rectangle corners
  buildRectangle(A, B, C, D, a_i, r_i);

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

// build rectangle for the possible motion path in the direction of polar angle a
// Width is carWidth_m, the length is (min_range + r)
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
void MotionComputer::buildRectangle(pcl::PointXY &A, pcl::PointXY &B, pcl::PointXY &C, pcl::PointXY &D,
  const float a, const float r) {
    register float Ar, Aa, Br, Ba, Cr, Ca, Dr, Da, tmp_cw2, tmp_far_a;

    tmp_cw2 = carWidth_m * carWidth_m * 0.25;

    // building polar coordinates
    Ar = sqrt( tmp_cw2 + r * r);
    tmp_far_a = asinf(carWidth_m * 0.5f / Ar);
    Aa = a - tmp_far_a;
    Ba = a + tmp_far_a;
    Br = Ar;
    Ca = a + RminHalfWidthAngle;
    Cr = sqrt(tmp_cw2 + min_range * min_range);
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
