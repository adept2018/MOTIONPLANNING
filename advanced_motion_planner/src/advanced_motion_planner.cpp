/** Advanced Motion Planner (AMP) source file
  * Originally created from Basic Motion Planner (BMP)

  * History:
  * 2019-03-20  Ported from BMP by Alexander Konovalenko
  * 2019-04-23  Successfully tested on the car. Lightning in the room
  *             can negatively affect the LIDAR!!!
  * 2019-06-07  Bug fixes.
  *
  **/

#include <advanced_motion_planner/advanced_motion_planner.h>
#include <advanced_motion_planner/amp_common.h>
#include <random>

int main(int argc, char** argv) {

    // Initialize ROS:
    ros::init(argc, argv, "advanced_motion_planner");
    ros::NodeHandle nh;
    MotionComputer motionComputer(nh);

    pubPoseVector = nh.advertise<geometry_msgs::PoseStamped>(POSE_VECTOR + OFFCAR, 10);
    pubCloudVisible = nh.advertise<sensor_msgs::PointCloud2>(CLOUD_VIS_NAME + OFFCAR, 10);
    pubCloudInvisible = nh.advertise<sensor_msgs::PointCloud2>(CLOUD_INVIS_NAME + OFFCAR, 10);
    pubNormDirection = nh.advertise<geometry_msgs::PoseStamped>(NORMDIR_VECTOR + OFFCAR, 10);
    pubAck = nh.advertise<ackermann_msgs::AckermannDriveStamped>(VESC_NAME, 10);
    #ifdef FUNCTIONAL_DEBUG_INFO
      pubPathCloud = nh.advertise<sensor_msgs::PointCloud2>(PATH_CLOUD + OFFCAR, 10);
    #endif
    #ifdef BACKWARD_MOTION
      std::random_device rd;  //Will be used to obtain a seed for the random number engine
      std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
      std::uniform_real_distribution<> rnd_SteerA(-SteeringAngleLimit, SteeringAngleLimit);
    #endif
    bool backward = false;

    ros::Rate rate(40.0);
    std::cout << "Running the advanced motion planner." << std::endl;

    while (nh.ok()) {
        ros::spinOnce();

        if (!motionComputer.computeMotion()) {
            //std::cout << "ERROR: Cannot compute motion." << std::endl;
            // we should not quit from application!
            //return -1;
        }

        float steeringAngle = 0.0f;
        geometry_msgs::PoseStamped outputMsg;

        if (!motionComputer.visibleCloud.empty()) {
          // prepare vectors and cloud for publications

          // Obtained angle and range (distance) from motion computer
          steeringAngle = motionComputer.direction[2];

          if (steeringAngle > SteeringAngleLimit) {
              steeringAngle = SteeringAngleLimit;
          } else if (steeringAngle < -SteeringAngleLimit) {
              steeringAngle = -SteeringAngleLimit;
          }
        } else {
          //there are no data points in visibleCloud
          //std::cout << "EMPTY visibleCloud!!!" << std::endl;
        }

        ackermann_msgs::AckermannDriveStamped ackMsg;

        // Specify speed (m/s), back-off is implemented, although direction is the same
        //ackMsg.drive.speed = DRIVE_SPEED_DEFAULT;
        // the following did not work perhaps due to light!!!
        //std::cout << "motionComputer.RAW.x: " << motionComputer.RAW.x << std::endl;
        if(motionComputer.RAW.x > NO_GO_MIN_DIST) {
          // drive forward
          ackMsg.drive.speed = DRIVE_SPEED_DEFAULT;
          ackMsg.drive.steering_angle = steeringAngle;
          pubAck.publish(ackMsg);
          backward = false;
        } else {
          // backward move
          #ifdef BACKWARD_MOTION
            ackMsg.drive.speed = BACK_SPEED_DEFAULT;
            steeringAngle = rnd_SteerA(gen);
            ackMsg.drive.steering_angle = steeringAngle;
            // publish speed and steering anlge:
            pubAck.publish(ackMsg);
            backward = true;
            // drive certain amount of distance or time
            ros::Duration(fabsf(BACK_DISTANCE / BACK_SPEED_DEFAULT)).sleep();
            std::cout << ">>>>>>>>>>>>>>>>>>> BACK OFF!!!" << std::endl;
          #else
            // no backward motion is enabled, but stop the car!
            ackMsg.drive.speed = 0.0f;
            ackMsg.drive.steering_angle = steeringAngle;
            pubAck.publish(ackMsg);
            backward = false;
            std::cout << ">>>>>>>>>>>>>>>>>>> STOP THE CAR!!!" << std::endl;
            ros::Duration(1.0f).sleep();
          #endif
        }


        // CPU consuming things are moved hereafter in order to prioritize turn/speed publishing

        #ifdef DEBUG2
          std::cout << "AMP steeringAngle [deg] & speed [m/s]:\t" << RAD2DEG(steeringAngle) \
            << "\t" << ackMsg.drive.speed << std::endl;
        #endif

        if (!motionComputer.invisibleCloud.empty()) {
            // Non visible point cloud from lidar
            sensor_msgs::PointCloud2 pclmsg;
            pcl::toROSMsg(motionComputer.invisibleCloud, pclmsg);
            pclmsg.header.frame_id = AMP_NAME;
            pubCloudInvisible.publish(pclmsg);
        }

        if (!motionComputer.visibleCloud.empty()) {
            outputMsg.header.frame_id = AMP_NAME;

            // this is nomral to the front bumper of the car or?
            outputMsg.pose.position.x = 0.0f;
            outputMsg.pose.position.y = 0.0f;
            outputMsg.pose.position.z = 0.0f;

            // this is sugested direction to turn (wheels steering limit is applied)
            pcl::PointXY xy = AMP_utils::polar2PointXY(motionComputer.RAW.x, steeringAngle);
            //std::cout << "Check steeringAngle: " << RAD2DEG(steeringAngle) << std::endl;
            if(backward) {
              outputMsg.pose.orientation.x = -xy.x;
              outputMsg.pose.orientation.y = -xy.y;
            } else {
              outputMsg.pose.orientation.x = xy.x;
              outputMsg.pose.orientation.y = xy.y;
            }

            outputMsg.pose.orientation.z = 0.0f;
            outputMsg.pose.orientation.w = 0.0f;

            //publish vectors and cloud
            pubPoseVector.publish(outputMsg);

            // Visible point cloud from lidar
            sensor_msgs::PointCloud2 pclmsg;
            pcl::toROSMsg(motionComputer.visibleCloud, pclmsg);
            pclmsg.header.frame_id = AMP_NAME;
            pubCloudVisible.publish(pclmsg);

            #ifdef FUNCTIONAL_DEBUG_INFO
              // Visualize calculated rectangular path for the car
              // note that it can differ from the actual direction adjusted by SteeringAngleLimit
              sensor_msgs::PointCloud2 ppclmsg;
              pcl::toROSMsg(motionComputer.pathCloud, ppclmsg);
              ppclmsg.header.frame_id = AMP_NAME;
              pubPathCloud.publish(ppclmsg);
            #endif
        }

        // Vector showing forward direction
        geometry_msgs::PoseStamped direction;
        direction.header.frame_id = AMP_NAME;

        direction.pose.position.x = 0.0f;
        direction.pose.position.y = 0.0f;
        direction.pose.position.z = 0.0f;

        direction.pose.orientation.x = 1.0f;
        direction.pose.orientation.y = 0.0f;
        direction.pose.orientation.z = 0.0f;
        direction.pose.orientation.w = 0.0f;
        pubNormDirection.publish(direction);

        rate.sleep();
    }
}
