#pragma once

#include "Planner.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "fcl/config.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/common/types.h"

class GlobalPlanner
{
private:
    static GlobalPlanner* inst_;
    Arm* CArm;
    ros::NodeHandle n;
    ros::Subscriber human_sub;
public:
    static LocalPlanner* GetLocalPlanner(Arm* carm);
    LocalPlanner(Arm* carm);
    ~LocalPlanner();

    void UpdateState(void);
    void APF_acc(void);
    float Attractor(float x, float x_dot);
    float Distractor(float x, float x_dot);
    void CreateFCL(void);
    void HumanCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void InitialROS(void);
    void Ros_spin(void);
    void StopROS(void);

    Eigen::Vector3f curr;
    Eigen::Vector3f last;
    Eigen::Vector3f goal;
    vector<Eigen::Vector3f> curr_obs;
    vector<Eigen::Vector3f> last_obs;
    float dt;
    Eigen::Vector3f acc;
    vector<Eigen::Vector3f> human;
    vector<int> human_IDlist;
    // vector<std::shared_ptr<fcl::CollisionGeometry>> human_shape;
    vector<fcl::CollisionObjectf> human_body;
    // head, body, r_arm1, r_arm2, r_palm, l_arm1, l_arm2, l_palm
};
