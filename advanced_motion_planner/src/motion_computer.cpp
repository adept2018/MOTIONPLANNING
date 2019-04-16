#include <advanced_motion_planner/motion_computer.h>

MotionComputer::MotionComputer(ros::NodeHandle &nh) {
    m_subscriber = nh.subscribe("/scan", 10, &MotionComputer::scanCallBack, this);
}

void MotionComputer::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
    m_laser_scan = *scan;
    m_scan_queue.push(m_laser_scan);
}

bool MotionComputer::computeMotion() {
    while (!m_scan_queue.empty()) {
        sensor_msgs::LaserScan scan = m_scan_queue.front();
        m_scan_queue.pop();

        cloud.clear();
        cloud = laserScanToPointCloud.scanToCloud(scan);

        // TODO:
        // followTheWall();

        float theta = 0;

        // Sum all angles
        for (uint16_t i = 0; i < cloud.size(); ++i) {
            float x = cloud.points[i].x;
            float y = cloud.points[i].y;
            theta += atanf(y / x);
        }

        // Dived angle by number of points
        float theta_w = theta / cloud.size();

        // Offset to turn away from obstacle
        float offset = 0.52f;

        // Decide which way to turn away from obstacle
        if (theta_w < 0.0f) {
            theta_w += offset;
        }
        else {
            theta_w += -offset;
        }

        direction.x = cosf(theta_w);
        direction.y = sinf(theta_w);
        direction.theta = theta_w;
    }
    return true;
}
