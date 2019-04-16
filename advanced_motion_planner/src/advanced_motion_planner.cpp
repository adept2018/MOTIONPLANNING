#include <advanced_motion_planner/advanced_motion_planner.h>

int main(int argc, char** argv) {

    // Initialize ROS:
    ros::init(argc, argv, "advanced_motion_planner");
    ros::NodeHandle nh;
    MotionComputer motionComputer(nh);

    pubPose = nh.advertise<geometry_msgs::PoseStamped>("bmp/pose", 10);
    pubCloudVisible = nh.advertise<sensor_msgs::PointCloud2>("bmp/cloud/visible", 10);
    pubDirection = nh.advertise<geometry_msgs::PoseStamped>("bmp/direction", 10);
    pubAck = nh.advertise<ackermann_msgs::AckermannDriveStamped>("vesc/high_level/ackermann_cmd_mux/input/default", 10);

    // frequency of ros i.e. the while loop
    // TODO: connect it with lidar?
    ros::Rate rate(40.0f);
    std::cout << "Running the advanced motion planner.\n";

    while (nh.ok()) {
        //TODO: investigate this
        ros::spinOnce();

        if (!motionComputer.computeMotion()) {
            std::cout << "ERROR: Cannot compute motion.\n";
            return -1;
        }

        if (!motionComputer.getCloud().empty()) {

            // Computed direction
            geometry_msgs::PoseStamped outputMsg;
            outputMsg.header.frame_id = "amp";

            outputMsg.pose.position.x = 0;
            outputMsg.pose.position.y = 0;
            outputMsg.pose.position.z = 0;

            const math::vec2 direction(motionComputer.getDirection());
            outputMsg.pose.orientation.x = direction.x;
            outputMsg.pose.orientation.y = direction.y;
            outputMsg.pose.orientation.z = 0;
            outputMsg.pose.orientation.w = 0;

            pubPose.publish(outputMsg);

            // Visible point cloud from lidar
            sensor_msgs::PointCloud2 pclmsg;
            pcl::toROSMsg(motionComputer.getCloud(), pclmsg);
            pclmsg.header.frame_id = "amp";
            pubCloudVisible.publish(pclmsg);
        }

        // normal from car to forward direction
        geometry_msgs::PoseStamped direction;
        direction.header.frame_id = "amp";

        direction.pose.position.x = 0;
        direction.pose.position.y = 0;
        direction.pose.position.z = 0;

        direction.pose.orientation.x = 1;
        direction.pose.orientation.y = 0;
        direction.pose.orientation.z = 0;
        direction.pose.orientation.w = 0;
        pubDirection.publish(direction);

        //message to the controller
        ackermann_msgs::AckermannDriveStamped ackMsg;
        // 0 or computed angle from motionComputer
        ackMsg.drive.steering_angle = motionComputer.getTurnAngle();
        // Use fixed speed (m/s)
        // TODO: try negative velocity
        ackMsg.drive.speed = motionComputer.getVelocityAmplitude();

        pubAck.publish(ackMsg);

        //TODO: investigate this
        rate.sleep();
    }
}
