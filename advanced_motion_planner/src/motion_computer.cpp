#include <advanced_motion_planner/motion_computer.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <queue>
#include <algorithm>

#define PI atan(1)*4;
#define BUCKET_COUNT 256
#define N_NEIGHBOURS 2

using namespace std;

bool sort_by_depth(clusterDepth & c1, clusterDepth & c2){
    return c1.dist > c2.dist;
}

/*! \enum class status
 *
 *  Detailed description
 */
enum status { CORE, BORDER, NOISE, NONE };

class Point{
    public:
        Point(float _x, float _y) : x(_x), y(_y){
            type = NONE;
        };
        enum status type;
        float dist2(const Point &p){
            return (x - p.x) * (x - p.x) + (y - p.y) * (y - p.y);
        }
        friend std::ostream & operator<< (std::ostream &out, const Point &p){
            out << "(" << p.x << ", " << p.y << ")";
            return out;
        } 
    private:
        float x;
        float y;
};

std::queue<int> getNeighbors(Point &p1, std::vector<Point> &data, const float eps){
    std::queue<int> neighbors;
    for(int i=0; i<data.size();i++){
        if( &p1 == &data[i] ) continue;
        if( p1.dist2(data[i]) <= eps*eps)
            neighbors.push(i);
    }
    return neighbors;
}

std::vector<std::vector<int> > dbscan(std::vector<Point> &data, const float eps, const int minP){
    std::vector<bool> visited(data.size(), false);
    std::vector<std::vector<int> > clusters;
    for (int i = 0; i < data.size(); ++i) {
        if(visited[i]) continue;
        visited[i] = true;    
        std::queue<int> nSet = getNeighbors(data[i], data, eps);
        if (nSet.size() < minP){
            data[i].type = NOISE;
        }
        else{
            clusters.push_back(std::vector<int>());
            clusters.back().push_back(i);
            data[i].type = CORE;
            while(!nSet.empty()){
                int j = nSet.front();
                nSet.pop();
                if(data[j].type != CORE && data[j].type != BORDER){
                    data[j].type = BORDER;
                    clusters.back().push_back(j);
                }
                if(visited[j]) continue;

                visited[j] = true;    
                std::queue<int> npSet = getNeighbors(data[j], data, eps);
                if(npSet.size() >= minP){
                    data[j].type = CORE;
                    while(!npSet.empty()){
                        nSet.push(npSet.front());
                        npSet.pop();
                    }
                }
            }
        }
    }
    return clusters;
}

void MotionComputer::imageDepthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	float *depth = (float *) (&msg->data[0]);
	/* int u = msg->width/2; */
	/* int v = msg->height/2; */
	/* int centerIndex = u + msg->width * v; */

	int start_y = msg->height/3 * msg->width;
	int end_y = msg->height/3 * 2 * msg->width;
	int bucket_width =  msg->width / BUCKET_COUNT;
	float number_of_points =  (end_y - start_y) * bucket_width;
	std::vector<float> depth_vec(BUCKET_COUNT, 0);
	std::vector<clusterDepth> new_depth_vec;
    std::vector<unsigned int> n_point(BUCKET_COUNT, 1);
    std::vector<Point> scan_points;

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
        else if (depth[i] >= 5 && depth[i] < 20){
            depth_vec[vec_index] += depth[i];
            n_point[vec_index]++;
            /* ROS_INFO("FARHAAAD, %f", depth[i]); */
        } 
	}

    /* ROS_INFO("ADRIAAAANNNNN, %f", number_of_points); */
	for(int i = 0; i < depth_vec.size(); i++){
        /* ROS_INFO("Point[%d] = %f", i, depth_vec[i] / n_point[i]); */
        depth_vec[i]/=n_point[i];
        scan_points.push_back(Point(i*0.1, depth_vec[i]));
    }

    std::vector<std::vector<int> > clusters = dbscan(scan_points, 0.15, N_NEIGHBOURS);
    /* ROS_INFO("N clusters: %d", clusters.size()); */
    for (int i =0 ; i < clusters.size(); i++){
        /* ROS_INFO("p in c %d %d", i, clusters[i].size()); */
        std::sort(clusters[i].begin(), clusters[i].end());
        new_depth_vec.push_back(clusterDepth(clusters[i].size(), clusters[i][0], depth_vec[clusters[i][0]]));
    }

    depth_queue.push(new_depth_vec);
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
        std::vector<clusterDepth> depthCloud = depth_queue.front();
        while (!depth_queue.empty())
            depth_queue.pop();
        /* invisibleCloud = laserScanToPointCloud.scanToCloud(scan, false); */
        float max_dist = -1;
        int max_dist_idx = -1;
        /* for(int i = 3; i < depthCloud.size() - 3; i++){ */
        /*     if(depthCloud[i] > max_dist){ */
        /*         max_dist = depthCloud[i]; */
        /*         max_dist_idx = i; */
        /*     } */
        /* } */

        std::sort(depthCloud.begin(), depthCloud.end(), sort_by_depth);
        for(int i = 0; i < depthCloud.size(); i++){
            if(depthCloud[i].size > 20 && depthCloud[i].dist > 0.35)
            {
                max_dist_idx = i;
                break;
            }
            /* if(depthCloud[i] > max_dist){ */
            /*     max_dist = depthCloud[i]; */
            /*     max_dist_idx = i; */
            /* } */
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

            /* float theta_w = 35 + 110.0/32.0 + 110.0/16.0*max_dist_idx -90; */
            /* /1* ROS_INFO("ANGLE %f", theta_w); *1/ */
            /* /1* printf("\rANGLE %f", theta_w); *1/ */
            /* direction.push_back(max_dist * cos(theta_w)); */
            /* direction.push_back(max_dist * sin(theta_w)); */
            /* direction.push_back(-theta_w); */
            /* direction.push_back(max_dist); */
        float theta_w = 0;
        if(max_dist_idx < 0){
            ROS_INFO("HASSSSAN RIDI");
            theta_w = -35;
            /* ROS_INFO("ANGLE %f", theta_w); */
            /* printf("\rANGLE %f", theta_w); */
            direction.push_back(0);
            direction.push_back(0);
            direction.push_back(theta_w);
            direction.push_back(0);
        }
        else{
            theta_w = 35 + 110.0/(2*BUCKET_COUNT) + 110.0/BUCKET_COUNT*(depthCloud[max_dist_idx].first_index +depthCloud[max_dist_idx].size/2.0) -90;
            /* ROS_INFO("ANGLE %f", theta_w); */
            /* printf("\rANGLE %f", theta_w); */
            direction.push_back(depthCloud[max_dist_idx].dist * cos(theta_w));
            direction.push_back(depthCloud[max_dist_idx].dist * sin(theta_w));
            direction.push_back(-theta_w);
            direction.push_back(depthCloud[max_dist_idx].dist);
        }
        /* } */ 
    }
    return true;
}
