#include <advanced_motion_planner/motion_computer.h>

MotionComputer::MotionComputer(ros::NodeHandle &nh) {
    m_subscriber = nh.subscribe("/scan", 10, &MotionComputer::scanCallBack, this);

    WallFollower wallFollower(const float& K_p, const float& K_i, const float& K_d, const float& dt, const float& min, const float& max);
}

void MotionComputer::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
    m_laser_scan = *scan;
    m_scan_queue.push(m_laser_scan);
}

bool MotionComputer::computeMotion() {
    parameters.update();

    while (!m_scan_queue.empty()) {
        sensor_msgs::LaserScan scan = m_scan_queue.front();
        m_scan_queue.pop();

        map.clear();
        observation_points.clear();
        wall_outline.clear();

        // Create map
        map = laserScanToPointCloud.scanToCloud(scan, 0, scan.ranges.size(), parameters.min_range, parameters.max_range, parameters.lidar_offset, true);

        // Isolate observation points for the wall follower
        uint16_t min_observation_point = static_cast<uint16_t>(cloud.size() * parameters.min_observation);
        uint16_t max_observation_point = static_cast<uint16_t>(cloud.size() * parameters.max_observation);
        observation_points = laserScanToPointCloud.scanToCloud(scan, min_observation_point, max_observation_point, parameters.min_range, parameters.max_range, parameters.lidar_offset, false);

        if (!wallFollower.followTheWall(observation_points, )) {
            std::cerr << "Failed to follow the wall - driving forward with reduced speed" << std::endl;
            ackMsg.steering_angle = 0.0f;
            ackMsg.speed = parameters.min_speed;
            direction.x = 0.0f;
            direction.y = 0.0f;
            return true;
        }

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


        pcl::PointCloud<pcl::PointXYZ> line;
        pcl::PointXYZ point;
        for (uint16_t i = parameters.min_line_x; i < parameters.max_line_x ++i) {
            point.x = i;
            point.y = m_a * i + m_b;
            point.z = 0;
            line.push_back(point);
        }


        direction.x = cosf(theta_w);
        direction.y = sinf(theta_w);
        direction.theta = theta_w;
    }
    return true;
}
