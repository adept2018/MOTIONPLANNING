/** Advanced Motion Planner (AMP) source file
  * Originally created from Basic Motion Planner (BMP)

  * History:
  * 2019-03-20  Ported from BMP by Alexander Konovalenko
  * 2019-04-23  Successfully tested on the car. Lightning in the room
  *             can negatively affect the LIDAR!!!
  *
  **/

#include <advanced_motion_planner/advanced_motion_planner.h>
#include <advanced_motion_planner/amp_common.h>

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
    #ifdef FUNCTIONAL_DEBUG_INFO
      pubPathCloud = nh.advertise<sensor_msgs::PointCloud2>("amp/cloud/pathCloud", 10);
    #endif

    ros::Rate rate(40.0);
    std::cout << "Running the advanced motion planner." << std::endl;

    while (nh.ok()) {
        ros::spinOnce();

        if (!motionComputer.computeMotion()) {
            std::cout << "ERROR: Cannot compute motion." << std::endl;
            return -1;
        }

        float steeringAngle = 0.0f, dist = 0.0f;

        if (!motionComputer.visibleCloud.empty()) {

            // Obtained angle and range (distance) from motion computer
            steeringAngle = motionComputer.direction[2];
            dist = sqrt(motionComputer.direction[0] * motionComputer.direction[0] + \
            motionComputer.direction[1] * motionComputer.direction[1]);

            if (steeringAngle > SteeringAngleLimit) {
                steeringAngle = SteeringAngleLimit;
            }
            if (steeringAngle < -SteeringAngleLimit) {
                steeringAngle = -SteeringAngleLimit;
            }

            // Computed vectors
            geometry_msgs::PoseStamped outputMsg;
            outputMsg.header.frame_id = "amp";

            // this is nomral to the front bumper of the car or?
            outputMsg.pose.position.x = 0.0f;
            outputMsg.pose.position.y = 0.0f;
            outputMsg.pose.position.z = 0.0f;

            // this is sugested direction to turn (wheels steering limit is not applied)
            /*outputMsg.pose.orientation.x = motionComputer.direction[0];
            outputMsg.pose.orientation.y = motionComputer.direction[1];
            outputMsg.pose.orientation.z = 0.0f;
            outputMsg.pose.orientation.w = 0.0f; */

            // this is sugested direction to turn (wheels steering limit is applied)
            pcl::PointXY xy = AMP_utils::polar2PointXY(dist, steeringAngle);
            outputMsg.pose.orientation.x = xy.x;
            outputMsg.pose.orientation.y = xy.y;
            outputMsg.pose.orientation.z = 0.0f;
            outputMsg.pose.orientation.w = 0.0f;

            pubPose.publish(outputMsg);

            // Visible point cloud from lidar
            sensor_msgs::PointCloud2 pclmsg;
            pcl::toROSMsg(motionComputer.visibleCloud, pclmsg);
            pclmsg.header.frame_id = "amp";
            pubCloudVisible.publish(pclmsg);

            #ifdef FUNCTIONAL_DEBUG_INFO
              // Visualize calculated rectangular path for the car
              // note that it can differ from the actual direction adjusted by SteeringAngleLimit
              sensor_msgs::PointCloud2 ppclmsg;
              pcl::toROSMsg(motionComputer.pathCloud, ppclmsg);
              ppclmsg.header.frame_id = "amp";
              pubPathCloud.publish(ppclmsg);
            #endif
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
        // Specify speed (m/s), back-off is implemented, although direction is the same
        // the following did not work perhaps due to light!!!
        //ackMsg.drive.speed = (dist > NO_GO_MIN_DIST) ? DRIVE_SPEED_DEFAULT : BACK_SPEED_DEFAULT;
        ackMsg.drive.speed = DRIVE_SPEED_DEFAULT;

        pubAck.publish(ackMsg);

        #ifdef DEBUG2
          std::cout << "AMP steeringAngle:\t" << RAD2DEG(steeringAngle) << std::endl;
        #endif

        rate.sleep();
    }
}
