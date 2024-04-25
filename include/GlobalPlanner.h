#pragma once

#include "Planner.h"
#include "OctreeGen.h"
// #include "visualization_msgs/Marker.h"
// #include "visualization_msgs/MarkerArray.h"

// #include "fcl/config.h"
// #include "fcl/narrowphase/distance.h"
// #include "fcl/common/types.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// #include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// #include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

class GlobalPlanner
{
private:
    bool isStateValid(const ompl::base::State *state);
    ompl::base::OptimizationObjectivePtr getObjWithCostToGo(const ompl::base::SpaceInformationPtr& si);
    void SetStart(Eigen::Vector3f start);
    void SetGoal(Eigen::Vector3f goal);
    void Plan(void);
    void InitialROS(void);
    void Ros_spin(void);
    void StopROS(void);
    
    static GlobalPlanner* inst_;
    Arm* CArm;
    ompl::base::StateSpacePtr space;
    ompl::base::SpaceInformationPtr si;
    ompl::base::ProblemDefinitionPtr pdef;

    std::thread* ros_thread;
    ros::NodeHandle n;
    ros::Rate* loop_rate;
    ros::Publisher path_pub;
    ros::Publisher octree_pub;
    ros::Subscriber pointcloud_sub;
    
public:
    static GlobalPlanner* GetGlobalPlanner(Arm* carm);
    GlobalPlanner(Arm* carm);
    ~GlobalPlanner();

    Eigen::Vector3f curr;
    OctreeGen octreeGen;
    octomap::OcTree* static_ob;
};
