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
            float theta = 0.0, tmp;
            bool free_front = true;

            // Sum all angles
            for (int i = 0; i < numberOfPoints; i++) {
                float x = visibleCloud.points[i].x;
                float y = visibleCloud.points[i].y;
                tmp = atan(y / x);
                theta += tmp;
                // determine if there is an obsticle at the front:
                if(fabs(tmp) < safe_angle)
                   free_front = false;
            }

            // Dived angle by number of points, i.e. average angle
            float theta_w = theta / numberOfPoints;

            // Decide which way to turn away from obstacle
            if(!free_front) {
                theta_w += (theta_w < 0.0) ? turn_offset : -turn_offset;
            } else {
                theta_w = 0.0;  // go straigt
            }

            direction.push_back(cos(theta_w));
            direction.push_back(sin(theta_w));
            direction.push_back(theta_w);
        }
    }
    return true;
}
