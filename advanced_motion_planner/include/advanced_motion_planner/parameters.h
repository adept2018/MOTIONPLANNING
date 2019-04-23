#ifndef PARAMETERS_H
#define PARAMETERS_H

class Parameters {
private:
    ros::NodeHandle *nodeHandle;
    bool isNodeHandleSet;

public:
    float lidar_offset;
    float min_speed;
    float max_speed;
    float speed_incr;

    Parameters() {
        isNodeHandleSet = false;
    }

    void setNodeHandle(ros::NodeHandle *nh) {
        nodeHandle = nh;
        isNodeHandleSet = true;
    }

    void update() {
        if (!isNodeHandleSet) {
            return;
        }

        nodeHandle->param<float>("/advanced_motion_planner/lidar_offset", lidar_offset, 0.00f);
        nodeHandle->param<float>("/advanced_motion_planner/min_speed", min_speed, 0.30f);
        nodeHandle->param<float>("/advanced_motion_planner/max_speed", max_speed, 0.50f);
        nodeHandle->param<float>("/advanced_motion_planner/speed_incr", speed_incr, 0.05f);
    }
};

#endif //PARAMETERS_H
