#include <advanced_motion_planner/advanced_motion_planner.h>

int main(int argc, char** argv) {

    // Initialize ROS:
    ros::init(argc, argv, "advanced_motion_planner");
    ros::NodeHandle nh;
    MotionComputer motionComputer(nh);

    pubPose = nh.advertise<geometry_msgs::PoseStamped>("amp/pose", 10);
    pubCloudVisible = nh.advertise<sensor_msgs::PointCloud2>("amp/cloud/visible", 10);
    pubCloudInvisible = nh.advertise<sensor_msgs::PointCloud2>("amp/cloud/invisible", 10);
    pubDirection = nh.advertise<geometry_msgs::PoseStamped>("amp/direction", 10);
    pubAck = nh.advertise<ackermann_msgs::AckermannDriveStamped>("vesc/high_level/ackermann_cmd_mux/input/default", 10);

    ros::Rate rate(40.0);
    std::cout << "Running the advanced motion planner." << std::endl;

    while (nh.ok()) {
        ros::spinOnce();

        if (!motionComputer.computeMotion()) {
            std::cout << "ERROR: Cannot compute motion." << std::endl;
            return -1;
        }

        float steeringAngle = 0;
        if(motionComputer.direction.size() == 0) continue;
            steeringAngle = motionComputer.direction[2]*0.0174532;
            /* if (steeringAngle > 0.34) { */
            /*     steeringAngle = 0.34; */
            /* } */
            /* if (steeringAngle < -0.34) { */
            /*     steeringAngle = -0.34; */
            /* } */
            ROS_INFO("ANGLE2 %f", steeringAngle);
            /* printf("\rANGLE %f,in grad  %f", motionComputer.direction[2], steeringAngle); */

        if (!motionComputer.visibleCloud.empty()) {

            // Computed angle
            steeringAngle = motionComputer.direction[2];
            ROS_INFO("ANGLE %f", steeringAngle);
            if (steeringAngle > 0.34) {
                steeringAngle = 0.34;
            }
            if (steeringAngle < -0.34) {
                steeringAngle = -0.34;
            }
            ROS_INFO("ANGLE2 %f", steeringAngle);

            // Computed direction
            geometry_msgs::PoseStamped outputMsg;
            outputMsg.header.frame_id = "amp";

            outputMsg.pose.position.x = 0;
            outputMsg.pose.position.y = 0;
            outputMsg.pose.position.z = 0;

            outputMsg.pose.orientation.x = motionComputer.direction[0];
            outputMsg.pose.orientation.y = motionComputer.direction[1];
            outputMsg.pose.orientation.z = 0;
            outputMsg.pose.orientation.w = 0;

            pubPose.publish(outputMsg);

            // Visible point cloud from lidar
            sensor_msgs::PointCloud2 pclmsg;
            pcl::toROSMsg(motionComputer.visibleCloud, pclmsg);
            pclmsg.header.frame_id = "amp";
            pubCloudVisible.publish(pclmsg);
        }

        // Vector showing forward direction
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

        if (!motionComputer.invisibleCloud.empty()) {

            // Non visible point cloud from lidar
            sensor_msgs::PointCloud2 pclmsg;
            pcl::toROSMsg(motionComputer.invisibleCloud, pclmsg);
            pclmsg.header.frame_id = "amp";
            pubCloudInvisible.publish(pclmsg);
        }

        ackermann_msgs::AckermannDriveStamped ackMsg;

        // 0 or computed angle from motionComputer
        ackMsg.drive.steering_angle = steeringAngle;
        // Use fixed speed (m/s)
        ackMsg.drive.speed = 0.5;

        pubAck.publish(ackMsg);

        rate.sleep();
    }
}
