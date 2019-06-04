#include <advanced_motion_planner/motion_computer.h>

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
	std::vector<float> depth_vec(BUCKET_COUNT, 0);
    std::vector<unsigned int> n_point(BUCKET_COUNT, 1);

	for(int i = start_y; i < end_y; i++)
	{
		int vec_index = (i%msg->width) / bucket_width;
        if(depth[i] >= 0.35 && depth[i]< 5){
            /* ROS_INFO("HASSAAAAAN, %d", vec_index); */
            depth_vec[vec_index] += depth[i];
            n_point[vec_index]++;
            /* depth_vec[vec_index] /= number_of_points; */
        }
        else if(depth[i]< 0.35){ 
            /* ROS_INFO("HASSAAAAAN, %f", depth[i]); */
            n_point[vec_index]++;
        } 
        else if (depth[i] >= 5){
            depth_vec[vec_index] += 5;
            n_point[vec_index]++;
            /* ROS_INFO("FARHAAAD, %f", depth[i]); */
        } 
	}

    /* ROS_INFO("ADRIAAAANNNNN, %f", number_of_points); */
	for(int i = 0; i < depth_vec.size(); i++){
        /* ROS_INFO("Point[%d] = %f", i, depth_vec[i] / n_point[i]); */
        depth_vec[i]/=n_point[i];
    }
    depth_queue.push(depth_vec);
	/* ROS_INFO("Center dist. [%g]", depth[centerIndex]); */
	/* ROS_INFO("Center dist. [%g], %f, %d, %d", depth[centerIndex], depth[start_y + u], start_y, centerIndex); */
}

MotionComputer::MotionComputer(ros::NodeHandle &nh) {
    // scan_sub = nh.subscribe("/scan", 10, &MotionComputer::scanCallBack, this);
    //scan_sub = nh.subscribe("/odom_zed", 10, &MotionComputer::chatterCallback, this);
      scan_sub = nh.subscribe("/zed/zed_node/depth/depth_registered", 10, &MotionComputer::imageDepthCallback, this);
}

void MotionComputer::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
    laser_scan = *scan;
    scan_queue.push(laser_scan);
}

bool MotionComputer::computeMotion() {
    while (!depth_queue.empty()) {
        /* sensor_msgs::LaserScan scan = scan_queue.front(); */
        /* scan_queue.pop(); */
        /* depth_queue.pop(); */

        direction.clear();
        /* depthCloud.clear(); */
        /* invisibleCloud.clear(); */

        /* visibleCloud = laserScanToPointCloud.scanToCloud(scan, true); */
        std::vector<float> depthCloud = depth_queue.front();
        while (!depth_queue.empty())
            depth_queue.pop();
        /* invisibleCloud = laserScanToPointCloud.scanToCloud(scan, false); */
        float max_dist = 0;
        int max_dist_idx = 0;
        for(int i = 3; i < depthCloud.size() - 3; i++){
            if(depthCloud[i] > max_dist){
                max_dist = depthCloud[i];
                max_dist_idx = i;
            }
        }
        /* ROS_INFO("max index %d", max_dist_idx); */

        /* int numberOfPoints = visibleCloud.size(); */

        /* if (numberOfPoints > 0) { */
        /*     float theta = 0; */

        /*     // Sum all angles */
        /*     for (int i = 0; i < numberOfPoints; i++) { */
        /*         float x = visibleCloud.points[i].x; */
        /*         float y = visibleCloud.points[i].y; */
        /*         theta += atan(y / x); */
        /*     } */

        /*     // Dived angle by number of points */
        /*     float theta_w = theta / numberOfPoints; */

        /*     // Offset to turn away from obstacle */
        /*     float offset = 0.52; */

        /*     // Decide which way to turn away from obstacle */
        /*     if (theta_w < 0) { */
        /*         theta_w += offset; */
        /*     } */
        /*     else { */
        /*         theta_w += -offset; */
        /*     } */

            float theta_w = 35 + 110.0/32.0 + 110.0/16.0*max_dist_idx -90;
            /* ROS_INFO("ANGLE %f", theta_w); */
            /* printf("\rANGLE %f", theta_w); */
            direction.push_back(max_dist * cos(theta_w));
            direction.push_back(max_dist * sin(theta_w));
            direction.push_back(-theta_w);
            direction.push_back(max_dist);
        /* } */ }
    return true;
}
