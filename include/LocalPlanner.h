#pragma once

#include "Planner.h"
#include "GlobalPlanner.h"
#include "RMP.h"
#include "ActualNode.h"
#include "Kinematics.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "fcl/config.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/common/types.h"

class LocalPlanner
{
private:
    void UpdateJoints(void);
    void UpdateArmFCL(void);
    void UpdateFCL(void);
    void UpdateDis(void);
    std::shared_ptr<RMPRoot> getRMP(const Eigen::Vector3f& goal_pos, const Eigen::Vector4f& goal_ori);
    Eigen::Matrix<float, 6, 2> Dynamics(Eigen::Matrix<float, 6, 2>& s, std::shared_ptr<RMPRoot> root);
    void CollsionPair(const int& idx, fcl::CollisionObjectf& obj1, fcl::CollisionObjectf& obj2);

    void UpdateState(void);
    void APF_acc(void);
    float Attractor(float x, float x_dot);
    float Distractor(float x, float x_dot);
    void CreateFCL(void);
    void InitialROS(void);
    void Ros_spin(void);
    void StopROS(void);

    static LocalPlanner* inst_;
    Arm* CArm;
    OctreeGen* octreeGen;
    GlobalPlanner* GPlanner;
    ros::NodeHandle n;
    ros::Subscriber human_sub;
    ros::Subscriber ball_sub;
    std::thread* ros_thread;
    ros::Rate* loop_rate;
    vector<float> joint_angles;
    vector<Eigen::Vector3f> key_joint_pos;
    octomap::OcTree* curr_octree;
    Eigen::Vector3f ball_pos;
    std::shared_ptr<fcl::CollisionGeometryf> curr_tree;
    vector<fcl::CollisionObjectf> ObjRobot;  // link1, link2, body, mobile
    deque<vector<float>> Path;

    Eigen::Vector3f goal_pos;
    Eigen::Vector3f goal_ori;
    Eigen::Vector3f curr_pos;
    Eigen::Vector3f curr_ori;

public:
    static LocalPlanner* GetLocalPlanner(Arm* carm, OctreeGen* octreegen);
    LocalPlanner(Arm* carm, OctreeGen* octreegen);
    ~LocalPlanner();
    void Run(void);
    float Move(const Eigen::Vector3f& goal_pos, const Eigen::Vector4f& goal_ori);
    
    void HumanCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    Eigen::Quaternionf Vec2Q(const Eigen::Vector3f& vec);
    
    // armClient client_armR;
    Eigen::Matrix<float, 6, 2> state;
    // Eigen::Vector3f curr_goal_pos;
    // Eigen::Vector4f curr_goal_ori;
    vector<Eigen::Vector3f> obs_pos_list;
    vector<Eigen::Vector3f> link_pos_list;
    vector<float> length_list;

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
