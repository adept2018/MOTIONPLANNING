#include <advanced_motion_planner/advanced_motion_planner.h>

int main(int argc, char** argv) {

    // Initialize ROS:
    ros::init(argc, argv, "advanced_motion_planner");
    ros::NodeHandle nh;
    MotionComputer motionComputer(nh);

    m_pubPose = nh.advertise<geometry_msgs::PoseStamped>("amp/pose", 10);
    m_pubCloud = nh.advertise<sensor_msgs::PointCloud2>("amp/cloud/visible", 10);
    m_pubDirection = nh.advertise<geometry_msgs::PoseStamped>("amp/direction", 10);
    m_pubAck = nh.advertise<ackermann_msgs::AckermannDriveStamped>("vesc/high_level/ackermann_cmd_mux/input/default", 10);

    ros::Rate rate(40.0f);
    std::cout << "Running the advanced motion planner." << std::endl;

    while (nh.ok()) {
        ros::spinOnce();

        if (!motionComputer.computeMotion()) {
            std::cout << "ERROR: Cannot compute motion." << std::endl;
            return -1;
        }

        float steeringAngle = 0.0f;

        if (!motionComputer.cloud.empty()) {

            // Computed angle
            steeringAngle = motionComputer.direction.theta;
            if (steeringAngle > 0.34f) {
                steeringAngle = 0.34f;
            }
            if (steeringAngle < -0.34f) {
                steeringAngle = -0.34f;
            }

            // Computed direction
            geometry_msgs::PoseStamped outputMsg;
            outputMsg.header.frame_id = "amp";

            outputMsg.pose.position.x = 0.0f;
            outputMsg.pose.position.y = 0.0f;
            outputMsg.pose.position.z = 0.0f;

            outputMsg.pose.orientation.x = motionComputer.direction.x;
            outputMsg.pose.orientation.y = motionComputer.direction.y;
            outputMsg.pose.orientation.z = 0.0f;
            outputMsg.pose.orientation.w = 0.0f;

            m_pubPose.publish(outputMsg);
        }

        // Vector showing forward direction
        geometry_msgs::PoseStamped direction;
        direction.header.frame_id = "amp";

        direction.pose.position.x = 0.0f;
        direction.pose.position.y = 0.0f;
        direction.pose.position.z = 0.0f;

        direction.pose.orientation.x = 1.0f;
        direction.pose.orientation.y = 0.0f;
        direction.pose.orientation.z = 0.0f;
        direction.pose.orientation.w = 0.0f;
        m_pubDirection.publish(direction);

        ackermann_msgs::AckermannDriveStamped ackMsg;

        // 0 or computed angle from motionComputer
        ackMsg.drive.steering_angle = steeringAngle;
        // Use fixed speed (m/s)
        ackMsg.drive.speed = 0.5f;

        m_pubAck.publish(ackMsg);

        rate.sleep();
    }
}
