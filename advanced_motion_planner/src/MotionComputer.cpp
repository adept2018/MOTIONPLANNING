#include "advanced_motion_planner/MotionComputer.h"

#include <cmath>

MotionComputer::MotionComputer(ros::NodeHandle &nh) :
    mLidarToCloudConverter(),
    mTurnAngle(0.0f),
    mSafeRadius(0.75f),
    mVelocityAmplitude(0.5f) {
        mCloud.reserve(1081);
      mZones.reserve(100);
      mScanSub = nh.subscribe("/scan", 10, &MotionComputer::scanCallBack, this);
}

void MotionComputer::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
    mScanQueue.push(*scan);
}

bool MotionComputer::computeMotion() {

    // why while?
    while (!mScanQueue.empty()) {

        // pop doesn't pop, it just erases element
        sensor_msgs::LaserScan scan = mScanQueue.front();
        mScanQueue.pop(); // does it free the memory though?

        // we get the new scan
        mCloud = std::move(mLidarToCloudConverter.scanToCloud(scan));

        //here we start new implementation :)

        bool activeZone = false;
        Zone currentZone(math::vec2(0.0f));

        float maxFreeDistance(0.0f);
        uint8_t zoneWithMaxClearance = 0;
        uint8_t currentZoneWithMaxClearance = 0;

        for(const auto& point: mCloud) {

            if (point.x >= mSafeRadius) {
                if (!activeZone) {
                    currentZone.PointA = math::vec2(point.x, point.y);
                    activeZone = true;
                }
                if(maxFreeDistance < point.x){
                    maxFreeDistance = point.x;
                    currentZoneWithMaxClearance = mZones.size();
                }
            }
            else if (activeZone) {
                const float x1 = currentZone.PointA.x * cos(currentZone.PointA.y);
                const float y1 = currentZone.PointA.x * sin(currentZone.PointA.y);
                const float x2 = point.x * cos(point.y);
                const float y2 = point.x * sin(point.y);
                // check if the car can fit between the points of the zone
                if(pow(x1-x2,2.0f) + pow(y1-y2,2.0f) > 0.16f){
                    currentZone.PointB = math::vec2(point.x, point.y);
                    mZones.emplace_back(currentZone);
                    zoneWithMaxClearance = currentZoneWithMaxClearance;
                }
                activeZone = false;
                // if(maxFreeDistance < point.x){
                //     maxFreeDistance = point.x;
                // }

            }
        }

        //if we still have an active zone then we close it with the last element
        //of the cloud
        if (activeZone){

            const math::vec2 point(mCloud.back().x, mCloud.back().y);
            const float x1 = currentZone.PointA.x * cos(currentZone.PointA.y);
            const float y1 = currentZone.PointA.x * sin(currentZone.PointA.y);
            const float x2 = point.x * cos(point.y);
            const float y2 = point.x * sin(point.y);
            // check if the car can fit between the points of the zone
            if(pow(x1 - x2, 2.0f) + pow(y1 - y2, 2.0f) > 0.16f){
                currentZone.PointB = math::vec2(point.x, point.y);
                mZones.emplace_back(currentZone);
                zoneWithMaxClearance = currentZoneWithMaxClearance;
            }
            activeZone = false;
        }

        // Choose the zone to drive in or slow down
        if(mZones.empty()) {

            mVelocityAmplitude *= 0.25f; //decrease velocity to 25% of current
            mTurnAngle = 0.0f;
        }
        else{

            mVelocityAmplitude = 0.5f;
            mTurnAngle = 0.5f * (mZones[zoneWithMaxClearance - 1].PointB.y +
                        mZones[zoneWithMaxClearance - 1].PointA.y);
        }

        // old planner start
        // const uint32_t numberOfPoints = mCloud.size();
        //
        // if (!mCloud.empty()) {
        //
        //     // Divided angle by number of points
        //     float theta = 0.0f;
        //
        //     // Sum all angles
        //     for (int i = 0; i < numberOfPoints; i++) {
        //         const float x = mCloud.points[i].x;
        //         const float y = mCloud.points[i].y;
        //         theta += atanf(y / x);
        //     }
        //
        //     // average theta
        //     theta /= numberOfPoints;
        //
        //     // Offset to turn away from obstacle
        //     float offset = 0.52f;
        //
        //     // Decide which way to turn away from obstacle
        //     if (theta < 0.0f) {
        //         theta += offset;
        //     }
        //     else {
        //         theta += -offset;
        //     }
        //
        //     // these checks moved in here from planner
        //     // because this function is responsible for points
        //     // values
        //     if (theta > 0.34f) {
        //         theta = 0.34f;
        //     }
        //     if (theta < -0.34f) {
        //         theta = -0.34f;
        //     }
        //
        //     // mDirection.SetX(cosf(theta));
        //     // mDirection.SetY(sinf(theta));
        //     mTurnAngle = theta;
        // }
        // else{
        //     //don't move!
        // }
        // old planner end
    }
    return true;
}
