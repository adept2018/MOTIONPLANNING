#include <advanced_motion_planner/advanced_motion_planner.h>

int main(int argc, char** argv) {

    // Initialize ROS:
    ros::init(argc, argv, "advanced_motion_planner");
    ros::NodeHandle nh;
    MotionComputer motionComputer(nh);

    pubPose = nh.advertise<geometry_msgs::PoseStamped>("bmp/pose", 10);
    pubCloudVisible = nh.advertise<sensor_msgs::PointCloud2>("bmp/cloud/visible", 10);
    pubCloudInvisible = nh.advertise<sensor_msgs::PointCloud2>("bmp/cloud/invisible", 10);
    pubDirection = nh.advertise<geometry_msgs::PoseStamped>("bmp/direction", 10);
    pubAck = nh.advertise<ackermann_msgs::AckermannDriveStamped>("vesc/high_level/ackermann_cmd_mux/input/default", 10);

    ros::Rate rate(40.0f);
    std::cout << "Running the advanced motion planner.\n";

    while (nh.ok()) {
        ros::spinOnce();

        if (!motionComputer.computeMotion()) {
            std::cout << "ERROR: Cannot compute motion.\n";
            return -1;
        }

        if (!motionComputer.mCloud.empty()) {

            // Computed direction
            geometry_msgs::PoseStamped outputMsg;
            outputMsg.header.frame_id = "bmp";

            outputMsg.pose.position.x = 0;
            outputMsg.pose.position.y = 0;
            outputMsg.pose.position.z = 0;

            outputMsg.pose.orientation.x = motionComputer.getDirection().GetX();
            outputMsg.pose.orientation.y = motionComputer.getDirection().GetY();
            outputMsg.pose.orientation.z = 0;
            outputMsg.pose.orientation.w = 0;

            pubPose.publish(outputMsg);

            // Visible point cloud from lidar
            sensor_msgs::PointCloud2 pclmsg;
            pcl::toROSMsg(motionComputer.mCloud, pclmsg);
            pclmsg.header.frame_id = "bmp";
            pubCloudVisible.publish(pclmsg);
        }

        // Vector showing forward direction
        geometry_msgs::PoseStamped direction;
        direction.header.frame_id = "bmp";

        direction.pose.position.x = 0;
        direction.pose.position.y = 0;
        direction.pose.position.z = 0;

        direction.pose.orientation.x = 1;
        direction.pose.orientation.y = 0;
        direction.pose.orientation.z = 0;
        direction.pose.orientation.w = 0;
        pubDirection.publish(direction);

        ackermann_msgs::AckermannDriveStamped ackMsg;

        // 0 or computed angle from motionComputer
        ackMsg.drive.steering_angle = motionComputer.getTurnAngle();

        // Use fixed speed (m/s)
        ackMsg.drive.speed = motionComputer.getVelocityAmplitude();

        pubAck.publish(ackMsg);

        rate.sleep();
    }
}
