#include <advanced_motion_planner/advanced_motion_planner.h>

int main(int argc, char** argv) {

    // Initialize ROS:
    ros::init(argc, argv, "advanced_motion_planner");
    ros::NodeHandle nodeHandle;
    MotionComputer motionComputer(nodeHandle);

    m_pubAck = nodeHandle.advertise<ackermann_msgs::AckermannDriveStamped>("vesc/high_level/ackermann_cmd_mux/input/default", 10);
    m_pubPose = nodeHandle.advertise<geometry_msgs::PoseStamped>("amp/pose", 10);
    m_pubDirection = nodeHandle.advertise<geometry_msgs::PoseStamped>("amp/direction", 10);
    m_pubMinObservationDirection = nodeHandle.advertise<geometry_msgs::PoseStamped>("amp/min_observation_direction", 10);
    m_pubMaxObservationDirection = nodeHandle.advertise<geometry_msgs::PoseStamped>("amp/max_observation_direction", 10);
    m_pubMap = nodeHandle.advertise<sensor_msgs::PointCloud2>("amp/cloud/map", 10);
    m_pubObservedPoints = nodeHandle.advertise<sensor_msgs::PointCloud2>("amp/cloud/observed_points", 10);
    m_pubCarrot = nodeHandle.advertise<sensor_msgs::PointCloud2>("amp/cloud/carrot", 10);
    m_pubWallOutline = nodeHandle.advertise<sensor_msgs::PointCloud2>("amp/cloud/wall_outline", 10);

    ros::Rate rate(40.0f);
    std::cout << "Running the advanced motion planner." << std::endl;

    while (nodeHandle.ok()) {
        ros::spinOnce();

        if (!motionComputer.computeMotion()) {
            std::cout << "ERROR: Cannot compute motion." << std::endl;
            return -1;
        }

        // Publish ackermann msg
        ackermann_msgs::AckermannDriveStamped ackMsg;
        ackMsg.drive.steering_angle = motionComputer.ackMsg.steering_angle;
        ackMsg.drive.speed = motionComputer.ackMsg.speed;
        m_pubAck.publish(ackMsg);

        // Publish computed direction
        geometry_msgs::PoseStamped computedDirectionMsg;
        computedDirectionMsg.pose.position.x = 0.0f;
        computedDirectionMsg.pose.position.y = 0.0f;
        computedDirectionMsg.pose.position.z = 0.0f;
        computedDirectionMsg.pose.orientation.x = motionComputer.direction.x;
        computedDirectionMsg.pose.orientation.y = motionComputer.direction.y;
        computedDirectionMsg.pose.orientation.z = 0.0f;
        computedDirectionMsg.pose.orientation.w = 0.0f;
        computedDirectionMsg.header.frame_id = HEADER_FRAME_ID;
        m_pubPose.publish(computedDirectionMsg);

        // Publish vector showing forward direction
        geometry_msgs::PoseStamped forwardDirectionMsg;
        forwardDirectionMsg.pose.position.x = 0.0f;
        forwardDirectionMsg.pose.position.y = 0.0f;
        forwardDirectionMsg.pose.position.z = 0.0f;
        forwardDirectionMsg.pose.orientation.x = 1.0f;
        forwardDirectionMsg.pose.orientation.y = 0.0f;
        forwardDirectionMsg.pose.orientation.z = 0.0f;
        forwardDirectionMsg.pose.orientation.w = 0.0f;
        forwardDirectionMsg.header.frame_id = HEADER_FRAME_ID;
        m_pubDirection.publish(forwardDirectionMsg);

        // Publish vector showing min observation direction
        geometry_msgs::PoseStamped minObservationDirectionMsg;
        minObservationDirectionMsg.pose.position.x = 0.0f;
        minObservationDirectionMsg.pose.position.y = 0.0f;
        minObservationDirectionMsg.pose.position.z = 0.0f;
        minObservationDirectionMsg.pose.orientation.x = motionComputer.min_observation_direction.x;
        minObservationDirectionMsg.pose.orientation.y = motionComputer.min_observation_direction.y;
        minObservationDirectionMsg.pose.orientation.z = 0.0f;
        minObservationDirectionMsg.pose.orientation.w = 0.0f;
        minObservationDirectionMsg.header.frame_id = HEADER_FRAME_ID;
        m_pubMinObservationDirection.publish(minObservationDirectionMsg);

        // Publish vector showing min observation direction
        geometry_msgs::PoseStamped maxObservationDirectionMsg;
        maxObservationDirectionMsg.pose.position.x = 0.0f;
        maxObservationDirectionMsg.pose.position.y = 0.0f;
        maxObservationDirectionMsg.pose.position.z = 0.0f;
        maxObservationDirectionMsg.pose.orientation.x = motionComputer.max_observation_direction.x;
        maxObservationDirectionMsg.pose.orientation.y = motionComputer.max_observation_direction.y;
        maxObservationDirectionMsg.pose.orientation.z = 0.0f;
        maxObservationDirectionMsg.pose.orientation.w = 0.0f;
        maxObservationDirectionMsg.header.frame_id = HEADER_FRAME_ID;
        m_pubMaxObservationDirection.publish(maxObservationDirectionMsg);

        // Publish pointcloud map
        if (!motionComputer.map.empty()) {
            sensor_msgs::PointCloud2 mapMsg;
            pcl::toROSMsg(motionComputer.map, mapMsg);
            mapMsg.header.frame_id = HEADER_FRAME_ID;
            m_pubMap.publish(mapMsg);
        }

        // Publish observed points
        if (!motionComputer.observed_points.empty()) {
            sensor_msgs::PointCloud2 observedPointMsg;
            pcl::toROSMsg(motionComputer.observed_points, observedPointMsg);
            observedPointMsg.header.frame_id = HEADER_FRAME_ID;
            m_pubObservedPoints.publish(observedPointMsg);
        }

        // Publish carrot
        if (!motionComputer.carrot.empty()) {
            sensor_msgs::PointCloud2 carrotMsg;
            pcl::toROSMsg(motionComputer.carrot, carrotMsg);
            carrotMsg.header.frame_id = HEADER_FRAME_ID;
            m_pubCarrot.publish(carrotMsg);
        }

        // Publish estimated wall outline
        if (!motionComputer.wall_outline.empty()) {
            sensor_msgs::PointCloud2 wallOutlineMsg;
            pcl::toROSMsg(motionComputer.wall_outline, wallOutlineMsg);
            wallOutlineMsg.header.frame_id = HEADER_FRAME_ID;
            m_pubWallOutline.publish(wallOutlineMsg);
        }

        rate.sleep();
    }
}
