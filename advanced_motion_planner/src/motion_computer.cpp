#include <advanced_motion_planner/motion_computer.h>

MotionComputer::MotionComputer(ros::NodeHandle &nh) {
    mScanSub = nh.subscribe("/scan", 10, &MotionComputer::scanCallBack, this);
}

void MotionComputer::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
    mScanQueue.push(*scan);
}
Direction MotionComputer::getDirection(){
  return mDirection;
}

void MotionComputer::setDirection(Direction& dir){
  mDirection.X = dir.X;
  mDirection.Y = dir.Y;
  mDirection.Omega = dir.Omega;

}


bool MotionComputer::computeMotion() {

    while (!mScanQueue.empty()) {

        // pop doesn't pop, it just erases element
        sensor_msgs::LaserScan scan = mScanQueue.front();
        mScanQueue.pop(); // does it free the memory though?

        mCloud.clear();

        mCloud = mLaserScanToPointCloud.scanToCloud(scan, true);

        uint32_t numberOfPoints = mCloud.size();

        if (numberOfPoints > 0) {

            // Dived angle by number of points
            float theta_w = 0.0f;

            // Sum all angles
            for (int i = 0; i < numberOfPoints; i++) {
                const float x = mCloud.points[i].x;
                const float y = mCloud.points[i].y;
                theta_w += atan(y / x);
            }

            theta_w /= numberOfPoints;

            // Offset to turn away from obstacle
            float offset = 0.52;

            // Decide which way to turn away from obstacle
            if (theta_w < 0) {
                theta_w += offset;
            }
            else {
                theta_w += -offset;
            }

            mDirection.X = cos(theta_w);
            mDirection.Y = sin(theta_w);
            mDirection.Omega = theta_w;
        }
    }
    return true;
}
