#include "GlobalPlanner.h"

GlobalPlanner* GlobalPlanner::inst_ = nullptr;

GlobalPlanner* GlobalPlanner::GetGlobalPlanner(Arm* carm) {
    if (inst_ == nullptr)
        inst_ = new GlobalPlanner(carm);
    return inst_;
}

GlobalPlanner::GlobalPlanner(Arm* carm) : octreeGen() {
    CArm = carm;
    curr = Eigen::Vector3f(CArm->GetCurrentPosition(0), CArm->GetCurrentPosition(1), CArm->GetCurrentPosition(2));

    // Planning space
    space = ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(3));
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, 0.0);
    bounds.setLow(1, -1.0);
    bounds.setLow(2, -1.5);
    bounds.setHigh(0, 3.0);
    bounds.setHigh(1, 1.0);
    bounds.setHigh(2, 0.5);
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    // Space information
    si = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(space));
    si->setStateValidityChecker(std::bind(&GlobalPlanner::isStateValid, this, std::placeholders::_1));
    si->setup();

    // Problem definition
    pdef = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(si));
    pdef->setOptimizationObjective(getObjWithCostToGo(si));
    pdef->setOptimizationObjective(ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(si)));

    InitialROS();
}

GlobalPlanner::~GlobalPlanner() {
    StopROS();
    inst_ = nullptr;
}

bool GlobalPlanner::isStateValid(const ompl::base::State *state) {
    // const double *values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    return false;
}

ompl::base::OptimizationObjectivePtr GlobalPlanner::getObjWithCostToGo(const ompl::base::SpaceInformationPtr& si) {
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    obj->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
    return obj;
}

void GlobalPlanner::SetStart(Eigen::Vector3f start) {
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start_state(this->space);
    start_state->values[0] = start[0];
    start_state->values[1] = start[1];
    start_state->values[2] = start[2];
    pdef->clearStartStates();
    pdef->addStartState(start_state);
}

void GlobalPlanner::SetGoal(Eigen::Vector3f goal) {
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal_state(this->space);
    goal_state->values[0] = goal[0];
    goal_state->values[1] = goal[1];
    goal_state->values[2] = goal[2];
    pdef->clearGoal();
    pdef->setGoalState(goal_state);
}

void GlobalPlanner::Plan(void) {
    // set the planner
    ompl::geometric::InformedRRTstar* pType = new ompl::geometric::InformedRRTstar(si);
    pType->setRange(0.05);
    ompl::base::PlannerPtr planner(pType);
    planner->setProblemDefinition(pdef);
    planner->setup();

    ompl::base::PlannerStatus solved = planner->solve(1.0);
    if (solved) {
        ompl::geometric::PathGeometric* path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        path->interpolate();

        nav_msgs::Path msg;
        msg.header.frame_id = "base";
        msg.header.stamp = ros::Time::now();
        for (size_t i = 0; i < path->getStateCount(); i++) {
            const ompl::base::RealVectorStateSpace::StateType* state = path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = state->values[0];
            pose.pose.position.y = state->values[1];
            pose.pose.position.z = state->values[2];
            // pose.pose.orientation.x = 0;
            // pose.pose.orientation.y = 0;
            // pose.pose.orientation.z = 0;
            // pose.pose.orientation.w = 1;
            msg.poses.push_back(pose);
        }
        path_pub.publish(msg);
    }
    else {
        cout << "No solution found" << endl;
    }
}

void GlobalPlanner::InitialROS(void) {
    pointcloud_sub = n.subscribe("/points2", 1, &OctreeGen::PointCloud2Octree, &octreeGen);
    octree_pub = n.advertise<octomap_msgs::Octomap>("/octomap", 1);
    path_pub = n.advertise<nav_msgs::Path>("/end_path",1);
    loop_rate = new ros::Rate(30);
    ros_thread = new std::thread(&GlobalPlanner::Ros_spin, this);
}

void GlobalPlanner::Ros_spin(void) {
    while(ros::ok()){
        static_ob = octreeGen.tree;
        octree_pub.publish(octreeGen.octree_msg);
        ros::spinOnce();
        loop_rate->sleep();        
    }
}

void GlobalPlanner::StopROS(void) {
    pointcloud_sub.shutdown();
    octree_pub.shutdown();
    path_pub.shutdown();
    delete loop_rate;
}
