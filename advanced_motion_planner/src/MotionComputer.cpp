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

        //here we start our implementation :)

        const uint32_t numberOfPoints = mCloud.size();

        if (!mCloud.empty()) {

            // Divided angle by number of points
            float theta = 0.0f;

            // Sum all angles
            for (int i = 0; i < numberOfPoints; i++) {
                const float x = mCloud.points[i].x;
                const float y = mCloud.points[i].y;
                theta += atanf(y / x);
            }

            // average theta
            theta /= numberOfPoints;

            // Offset to turn away from obstacle
            float offset = 0.52f;

            // Decide which way to turn away from obstacle
            if (theta < 0.0f) {
                theta += offset;
            }
            else {
                theta += -offset;
            }

            // these checks moved in here from planner
            // because this function is responsible for points
            // values
            if (theta > 0.34f) {
                theta = 0.34f;
            }
            if (theta < -0.34f) {
                theta = -0.34f;
            }

            // mDirection.SetX(cosf(theta));
            // mDirection.SetY(sinf(theta));
            mTurnAngle = theta;
        }
        else{
            //don't move!
        }
    }
    return true;
}
