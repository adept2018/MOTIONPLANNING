#include <advanced_motion_planner/motion_computer.h>

MotionComputer::MotionComputer(ros::NodeHandle &nh) :
    mDirection(0.0f),
    mTurnAngle(0.0f),
    mVelocityAmplitude(0.5f) {
        mScanSub = nh.subscribe("/scan", 10, &MotionComputer::scanCallBack, this);
}

void MotionComputer::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
    mScanQueue.push(*scan);
}

// void MotionComputer::setDirection(vec2& dir){
//   mDirection.SetX(dir.GetX());
//   mDirection.SetY(dir.GetY());
// }

bool MotionComputer::computeMotion() {

    while (!mScanQueue.empty()) {

        // pop doesn't pop, it just erases element
        sensor_msgs::LaserScan scan = mScanQueue.front();
        mScanQueue.pop(); // does it free the memory though?

        mCloud.clear();

        mCloud = mLaserScanToPointCloud.scanToCloud(scan);

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
            float offset = 0.52f;

            // Decide which way to turn away from obstacle
            if (theta_w < 0) {
                theta_w += offset;
            }
            else {
                theta_w += -offset;
            }

            mDirection.SetX(cos(theta_w));
            mDirection.SetY(sin(theta_w));

            // these checks moved in here from planner
            if (theta_w > 0.34f) {
                theta_w = 0.34f;
            }
            if (theta_w < -0.34f) {
                theta_w = -0.34f;
            }
            mTurnAngle = theta_w;
        }
    }
    return true;
}
