#pragma once

#include "Planner.h"
#include "OctreeGen.h"

#include <airobots_calvin/ArmAction.h>
#include "Actions/ActionClient.h"

#include "fcl/config.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/geometry/octree/octree-inl.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/common/types.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// #include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// #include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

// class armClient : public Client<airobots_calvin::ArmAction>
// {
// public:
//     armClient(std::string name) : Client<airobots_calvin::ArmAction>(name) {}
//     void pushGoal(Goal g) {
//         std::lock_guard<std::mutex> lock(goalQ_mtx);
//         goalQ.push(g);
//         ROS_INFO("[%s] Current State: %s", servername.c_str(), ac.getState().toString().c_str());
//         while (ac.getState() == actionlib::SimpleClientGoalState::LOST || ac.getState() == actionlib::SimpleClientGoalState::ACTIVE || ac.getState() == actionlib::SimpleClientGoalState::PENDING) {
//             ROS_INFO("[%s] Waiting for action to finish.", servername.c_str());
//         }

//         goalQ_cv.notify_one();
//     }
// };

class GlobalPlanner
{
private:
    bool isStateValid(const ompl::base::State *state);
    bool isIKValid(vector<float> pos);
    ompl::base::OptimizationObjectivePtr getObjWithCostToGo(const ompl::base::SpaceInformationPtr& si);
    void InitialROS(void);
    void Ros_spin(void);
    void StopROS(void);
    
    static GlobalPlanner* inst_;
    Arm* CArm;
    ompl::base::StateSpacePtr space;
    ompl::base::SpaceInformationPtr si;
    ompl::base::ProblemDefinitionPtr pdef;
    // ompl::geometric::PathGeometric smooth_path;
    std::shared_ptr<fcl::CollisionGeometryf> static_tree;
    std::shared_ptr<fcl::CollisionGeometryf> endeffector;
    std::shared_ptr<fcl::CollisionGeometryf> body;
    std::shared_ptr<fcl::CollisionGeometryf> mobile;

    ros::NodeHandle n;
    std::thread* ros_thread;
    ros::Rate* loop_rate;
    ros::Publisher path_pub;
    ros::Publisher octree_pub;
    ros::Publisher marker_pub;
    ros::Subscriber pointcloud_sub;
    ros::Subscriber motor_sub;
    
public:
    static GlobalPlanner* GetGlobalPlanner(Arm* carm, OctreeGen* octreegen);
    GlobalPlanner(Arm* carm, OctreeGen* octreegen);
    ~GlobalPlanner();
    void SetStart(vector<float> start);
    void SetGoal(vector<float> goal);
    void Plan(void);
    deque<vector<float>> GetPath(void);
    Eigen::Vector3f Q2Euler(const Eigen::Quaternionf& q);

    vector<float> curr;
    OctreeGen* octreeGen;
    pcl::PointCloud<pcl::PointXYZ> static_cloud;
    octomap::OcTree* static_octree;
    deque<vector<float>> path_states;
    nav_msgs::Path path_msg;
    // armClient client_armR;
};


