#include <advanced_motion_planner/motion_computer.h>
#include <vector>

#define PI atan(1)*4;
#define BUCKET_COUNT 16

void MotionComputer::imageDepthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	float *depth = (float *) (&msg->data[0]);
	int u = msg->width/2;
	int v = msg->height/2;
	int centerIndex = u + msg->width * v;
		
	int start_y = msg->height/3 * msg->width;
	int end_y = msg->height/3 * 2 * msg->width;
	int bucket_width =  msg->width / BUCKET_COUNT;
	float number_of_points =  (end_y - start_y) * bucket_width;
	std::vector<float> depth_vec(BUCKET_COUNT);

	for(int i = start_y; i < end_y; i++)
	{
		int vec_index = i%msg->width / bucket_width;
		depth_vec[vec_index] += depth[i];  
		depth_vec[vec_index] /= number_of_points;
	}

	for(int i = 0; i < depth_vec.size(); i++)
		ROS_INFO("Point[%d] = %f", i, depth_vec[i]);
	ROS_INFO("Center dist. [%g]", depth[centerIndex]);
}

MotionComputer::MotionComputer(ros::NodeHandle &nh) {
    // scan_sub = nh.subscribe("/scan", 10, &MotionComputer::scanCallBack, this);
    //scan_sub = nh.subscribe("/odom_zed", 10, &MotionComputer::chatterCallback, this);
      scan_sub = nh.subscribe("/depth/depth_registered", 10, &MotionComputer::imageDepthCallback, this);
}

void MotionComputer::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
    laser_scan = *scan;
    scan_queue.push(laser_scan);
}

bool MotionComputer::computeMotion() {
    while (!scan_queue.empty()) {
        sensor_msgs::LaserScan scan = scan_queue.front();
        scan_queue.pop();

        direction.clear();
        visibleCloud.clear();
        invisibleCloud.clear();

        visibleCloud = laserScanToPointCloud.scanToCloud(scan, true);
        invisibleCloud = laserScanToPointCloud.scanToCloud(scan, false);

        int numberOfPoints = visibleCloud.size();

        if (numberOfPoints > 0) {
            float theta = 0;

            // Sum all angles
            for (int i = 0; i < numberOfPoints; i++) {
                float x = visibleCloud.points[i].x;
                float y = visibleCloud.points[i].y;
                theta += atan(y / x);
            }

            // Dived angle by number of points
            float theta_w = theta / numberOfPoints;

            // Offset to turn away from obstacle
            float offset = 0.52;

            // Decide which way to turn away from obstacle
            if (theta_w < 0) {
                theta_w += offset;
            }
            else {
                theta_w += -offset;
            }

            direction.push_back(cos(theta_w));
            direction.push_back(sin(theta_w));
            direction.push_back(theta_w);
        }
    }
    return true;
}
