#include <advanced_motion_planner/MotionComputer.h>

MotionComputer::MotionComputer(ros::NodeHandle &nh) :
    mLidarToCloudConverter(),
    mTurnAngle(0.0f),
    mVelocityAmplitude(0.5f) {
      mCloud.reserve(1081);
      mScanSub = nh.subscribe("/scan", 10, &MotionComputer::scanCallBack, this);
}

void MotionComputer::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
    mScanQueue.push(*scan);
}

bool MotionComputer::computeMotion() {

    while (!mScanQueue.empty()) {

        // pop doesn't pop, it just erases element
        sensor_msgs::LaserScan scan = mScanQueue.front();
        mScanQueue.pop(); // does it free the memory though?

        // we clear the previous scan
        mCloud.clear();

        // we get the new scan
        mCloud = mLidarToCloudConverter.scanToCloud(scan);

        const uint32_t numberOfPoints = mCloud.size();

        if (numberOfPoints > 0) {

            // Divided angle by number of points
            float theta_w = 0.0f;

            // Sum all angles
            for (int i = 0; i < numberOfPoints; i++) {
                const float x = mCloud.points[i].x;
                const float y = mCloud.points[i].y;
                theta_w += atanf(y / x);
            }

            // average theta
            theta_w /= numberOfPoints;

            // Offset to turn away from obstacle
            float offset = 0.52f;

            // Decide which way to turn away from obstacle
            if (theta_w < 0.0f) {
                theta_w += offset;
            }
            else {
                theta_w += -offset;
            }

            // these checks moved in here from planner
            // because this function is responsible for points
            // values
            if (theta_w > 0.34f) {
                theta_w = 0.34f;
            }
            if (theta_w < -0.34f) {
                theta_w = -0.34f;
            }

            // mDirection.SetX(cosf(theta_w));
            // mDirection.SetY(sinf(theta_w));
            mTurnAngle = theta_w;
        }
        else{
            //don't move!
        }
    }
    return true;
}
