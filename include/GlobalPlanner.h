#pragma once

#include "Planner.h"
#include "OctreeGen.h"
// #include "visualization_msgs/Marker.h"
// #include "visualization_msgs/MarkerArray.h"

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

class GlobalPlanner
{
private:
    bool isStateValid(const ompl::base::State *state);
    ompl::base::OptimizationObjectivePtr getObjWithCostToGo(const ompl::base::SpaceInformationPtr& si);
    void SetStart(vector<float> start);
    void SetGoal(vector<float> goal);
    void Plan(void);
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

    vector<float> curr;
    OctreeGen octreeGen;
    pcl::PointCloud<pcl::PointXYZ> static_cloud;
    octomap::OcTree* static_octree;
};
