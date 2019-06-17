#include <advanced_motion_planner/motion_computer.h>

MotionComputer::MotionComputer(ros::NodeHandle& nodeHandle) {
    m_subscriber = nodeHandle.subscribe("/scan", 10, &MotionComputer::scanCallBack, this);

    ros::NodeHandle* nh = &nodeHandle;
    parameters.setNodeHandle(nh);
    parameters.update();

    wallFollower.initialize(parameters.wall_distance, parameters.carrot_distance,
                              parameters.K_p, parameters.K_i, parameters.K_d,
                              parameters.min_direction, parameters.max_direction,
                              parameters.carrot_radius, parameters.min_wall_line, parameters.max_wall_line);
}

void MotionComputer::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
    m_laser_scan = *scan;
    m_scan_queue.push(m_laser_scan);
}

bool MotionComputer::computeMotion() {

    while (!m_scan_queue.empty()) {
        sensor_msgs::LaserScan scan = m_scan_queue.front();
        m_scan_queue.pop();

        map.clear();
        observed_points.clear();
        carrot.clear();
        wall_outline.clear();

        // Create map
        map = laserScanToPointCloud.scanToCloud(scan, 0, scan.ranges.size(), parameters.min_scan_range, parameters.max_scan_range, parameters.lidar_offset, true);

        // Isolate observation points for the wall follower
        uint16_t min_observation_point = static_cast<uint16_t>(scan.ranges.size() * parameters.min_observation);
        uint16_t max_observation_point = static_cast<uint16_t>(scan.ranges.size() * parameters.max_observation);
        observed_points = laserScanToPointCloud.scanToCloud(scan, min_observation_point, max_observation_point, parameters.min_scan_range, parameters.max_scan_range, parameters.lidar_offset, false);

        m_dt = scan.scan_time;
        std::cerr << "m_dt: " << m_dt << std::endl;

        if (!wallFollower.followTheWall(observed_points, m_dt)) {
            std::cerr << "Failed to follow the wall - driving forward with reduced speed" << std::endl;
            ackMsg.steering_angle = 0.0f;
            ackMsg.speed = parameters.min_speed;
            direction.x = 1.0f;
            direction.y = 1.0f;
            return true;
        }

        carrot = wallFollower.calculated_carrot;
        wall_outline = wallFollower.estimated_line;

        ackMsg.steering_angle = wallFollower.direction;
        direction.x = cosf(wallFollower.direction);
        direction.y = sinf(wallFollower.direction);
        ackMsg.speed = parameters.min_speed;

        return true;
    }

    return true;
}
