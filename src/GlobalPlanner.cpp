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
    InitialROS();
    
    // Create static octree
    while(!octreeGen.static_flag){};
    static_octree = octreeGen.ori_octree;
    fcl::OcTreef* tree = new fcl::OcTreef(std::shared_ptr<const octomap::OcTree>(static_octree));
    static_tree = std::shared_ptr<fcl::CollisionGeometryf>(tree);

    // Create endeffector
    endeffector = std::shared_ptr<fcl::CollisionGeometryf>(new fcl::Cylinderf(0.2 / 2, 0.28));
    body = std::shared_ptr<fcl::CollisionGeometryf>(new fcl::Boxf(80.0/1000, 260.0/1000, 350.0/1000));
    mobile = std::shared_ptr<fcl::CollisionGeometryf>(new fcl::Boxf(210.0/1000, 440.0/1000, 210.0/1000));

    // Planning space
    space = ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace());
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, 0.0);
    bounds.setLow(1, -1.0);
    bounds.setLow(2, -1.5);
    bounds.setHigh(0, 3.0);
    bounds.setHigh(1, 1.0);
    bounds.setHigh(2, 0.5);
    space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

    // Space information
    si = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(space));
    si->setStateValidityChecker(std::bind(&GlobalPlanner::isStateValid, this, std::placeholders::_1));
    si->setup();

    // Problem definition
    pdef = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(si));
    pdef->setOptimizationObjective(getObjWithCostToGo(si));
}

GlobalPlanner::~GlobalPlanner() {
    StopROS();
    inst_ = nullptr;
}

bool GlobalPlanner::isStateValid(const ompl::base::State *state) {
    const ompl::base::SE3StateSpace::StateType* SE3state = state->as<ompl::base::SE3StateSpace::StateType>();
    const ompl::base::RealVectorStateSpace::StateType* pos = SE3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
    const ompl::base::SO3StateSpace::StateType* rot = SE3state->as<ompl::base::SO3StateSpace::StateType>(1);

    Eigen::Quaternionf q(rot->w, rot->x, rot->y, rot->z);
    Eigen::Quaternionf p(0, 0, 0, 0.28 / 2);
    Eigen::Quaternionf rotated_p = q * p * q.inverse();
    Eigen::Vector3f p0 = rotated_p.vec();
    Eigen::Vector3f p1 = Eigen::Vector3f(pos->values[0], pos->values[1], pos->values[2]) + p0;

    Eigen::Matrix3f R = q.toRotationMatrix();
    Eigen::Vector3f euler = R.eulerAngles(2, 1, 0);

    fcl::CollisionObjectf treeObj(static_tree);
    fcl::CollisionObjectf endeffectorObj(endeffector);
    // fcl::CollisionObjectf bodyObj(body);
    // fcl::CollisionObjectf mobileObj(mobile);

    fcl::Vector3f te(p1[0], p1[1], p1[2]);
    fcl::Quaternionf re(q.w(), q.x(), q.y(), q.z());
    endeffectorObj.setTransform(re, te);

    fcl::CollisionRequestf requestType(1, false, 1, false);
    fcl::CollisionResultf collisionResult;
    fcl::collide(&endeffectorObj, &treeObj, requestType, collisionResult);

    return (!collisionResult.isCollision());
}

ompl::base::OptimizationObjectivePtr GlobalPlanner::getObjWithCostToGo(const ompl::base::SpaceInformationPtr& si) {
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    obj->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
    return obj;
}

void GlobalPlanner::SetStart(vector<float> start) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start_state(this->space);
    start_state->setXYZ(start[0], start[1], start[2]);
    float cr = cos(start[3] * 0.5);
    float sr = sin(start[3] * 0.5);
    float cp = cos(start[4] * 0.5);
    float sp = sin(start[4] * 0.5);
    float cy = cos(start[5] * 0.5);
    float sy = sin(start[5] * 0.5);
    float w = cr * cp * cy + sr * sp * sy;
    float x = sr * cp * cy - cr * sp * sy;
    float y = cr * sp * cy + sr * cp * sy;
    float z = cr * cp * sy - sr * sp * cy;
    start_state->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(x, y, z, w);
    pdef->clearStartStates();
    pdef->addStartState(start_state);
}

void GlobalPlanner::SetGoal(vector<float> goal) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal_state(this->space);
    goal_state->setXYZ(goal[0], goal[1], goal[2]);
    float cr = cos(goal[3] * 0.5);
    float sr = sin(goal[3] * 0.5);
    float cp = cos(goal[4] * 0.5);
    float sp = sin(goal[4] * 0.5);
    float cy = cos(goal[5] * 0.5);
    float sy = sin(goal[5] * 0.5);
    float w = cr * cp * cy + sr * sp * sy;
    float x = sr * cp * cy - cr * sp * sy;
    float y = cr * sp * cy + sr * cp * sy;
    float z = cr * cp * sy - sr * sp * cy;
    goal_state->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(x, y, z, w);
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

        // ompl::geometric::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
		// smooth_path = new ompl::geometric::PathGeometric(dynamic_cast<const ompl::geometric::PathGeometric&>(*pdef->getSolutionPath()));
		// pathBSpline->smoothBSpline(*smooth_path, 3);

        nav_msgs::Path msg;
        msg.header.frame_id = "base";
        msg.header.stamp = ros::Time::now();
        for (size_t i = 0; i < path->getStateCount(); i++) {
            const ompl::base::SE3StateSpace::StateType* state = path->getState(i)->as<ompl::base::SE3StateSpace::StateType>();
            const ompl::base::RealVectorStateSpace::StateType* pos = state->as<ompl::base::RealVectorStateSpace::StateType>(0);
            const ompl::base::SO3StateSpace::StateType* rot = state->as<ompl::base::SO3StateSpace::StateType>(1);
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = pos->values[0];
            pose.pose.position.y = pos->values[1];
            pose.pose.position.z = pos->values[2];
            pose.pose.orientation.x = rot->x;
            pose.pose.orientation.y = rot->y;
            pose.pose.orientation.z = rot->z;
            pose.pose.orientation.w = rot->w;
            msg.poses.push_back(pose);
        }
        path_pub.publish(msg);
    }
    else {
        cout << "No solution found" << endl;
    }
}

void GlobalPlanner::InitialROS(void) {
    pointcloud_sub = n.subscribe("/points2", 1, &OctreeGen::PointCloudCallback, &octreeGen);
    octree_pub = n.advertise<octomap_msgs::Octomap>("/octomap", 1);
    path_pub = n.advertise<nav_msgs::Path>("/end_path",1);
    loop_rate = new ros::Rate(30);
    ros_thread = new std::thread(&GlobalPlanner::Ros_spin, this);
}

void GlobalPlanner::Ros_spin(void) {
    while(ros::ok()){
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
