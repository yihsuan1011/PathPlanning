#include "GlobalPlanner.h"

GlobalPlanner* GlobalPlanner::inst_ = nullptr;

GlobalPlanner* GlobalPlanner::GetGlobalPlanner(Arm* carm, OctreeGen* octreegen) {
    if (inst_ == nullptr)
        inst_ = new GlobalPlanner(carm, octreegen);
    return inst_;
}

GlobalPlanner::GlobalPlanner(Arm* carm, OctreeGen* octreegen)/* : client_armR("armR")*/ {
    CArm = carm;
    octreeGen = octreegen;
    // curr = {CArm->GetCurrentPosition(0), CArm->GetCurrentPosition(1), CArm->GetCurrentPosition(2), 
    //         CArm->GetCurrentOrientation(0), CArm->GetCurrentOrientation(1), CArm->GetCurrentOrientation(2), CArm->GetCurrentOrientation(3)};
    // curr = Eigen::Vector3f(CArm->GetCurrentPosition(0), CArm->GetCurrentPosition(1), CArm->GetCurrentPosition(2));
    InitialROS();
    // client_armR.run();

    // Create static octree
    while(!octreeGen->static_flag) {
        // ROS_INFO("static_flag: %d", octreeGen.static_flag);
    };
    static_octree = octreeGen->ori_octree;
    fcl::OcTreef* tree = new fcl::OcTreef(std::shared_ptr<const octomap::OcTree>(static_octree));
    static_tree = std::shared_ptr<fcl::CollisionGeometryf>(tree);

    // Create endeffector
    endeffector = std::shared_ptr<fcl::CollisionGeometryf>(new fcl::Cylinderf(0.2 / 2, 0.32));
    body = std::shared_ptr<fcl::CollisionGeometryf>(new fcl::Boxf(80.0/1000, 260.0/1000, 350.0/1000));
    mobile = std::shared_ptr<fcl::CollisionGeometryf>(new fcl::Boxf(210.0/1000, 440.0/1000, 210.0/1000));

    // Planning space
    space = ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace());
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, -1.0);
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
    ROS_INFO("Space information set up");

    // Problem definition
    pdef = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(si));
    pdef->setOptimizationObjective(getObjWithCostToGo(si));
    ROS_INFO("Problem definition set up");

    path_states = {};
}

GlobalPlanner::~GlobalPlanner() {
    StopROS();
    delete ros_thread;
    inst_ = nullptr;
}

bool GlobalPlanner::isStateValid(const ompl::base::State *state) {
    const ompl::base::SE3StateSpace::StateType* SE3state = state->as<ompl::base::SE3StateSpace::StateType>();
    const ompl::base::RealVectorStateSpace::StateType* pos = SE3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
    const ompl::base::SO3StateSpace::StateType* rot = SE3state->as<ompl::base::SO3StateSpace::StateType>(1);

    Eigen::Quaternionf q(rot->w, rot->x, rot->y, rot->z);
    // Eigen::Quaternionf p(0, 0, 0, 0.32 / 2);
    // Eigen::Quaternionf rotated_p = q * p * q.inverse();

    // Eigen::Vector3f p0 = rotated_p.vec();
    // Eigen::Vector3f p1 = Eigen::Vector3f(pos->values[0], pos->values[1], pos->values[2]) + p0;

    Eigen::Matrix3f R = q.toRotationMatrix();
    Eigen::Matrix3f Re = R * Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY());
    Eigen::Quaternionf qe(Re);
    // Eigen::Vector3f euler = R.eulerAngles(2, 1, 0);
    Eigen::Vector3f euler = Q2Euler(q);
    Eigen::Vector3f m = Re * Eigen::Vector3f(0, 0, 0.32) + Eigen::Vector3f(pos->values[0], pos->values[1], pos->values[2]);

    vector<float> po = {pos->values[0], pos->values[1], pos->values[2], euler[0], euler[1], euler[2]};
    bool isIKV = isIKValid(po);
    ROS_INFO("isIKV: %d", isIKV);
    if (!isIKV)
        return false;

    fcl::CollisionObjectf treeObj(static_tree);
    fcl::CollisionObjectf endeffectorObj(endeffector);
    fcl::CollisionObjectf bodyObj(body);
    fcl::CollisionObjectf mobileObj(mobile);

    fcl::Vector3f te(m[0], m[1], m[2]);
    fcl::Vector3f tb((0.0)/2/1000, (0.0)/2/1000, -(350.0)/2/1000);
    fcl::Vector3f tm((0.0)/2/1000, (0.0)/2/1000, -(210.0)/2/1000-500.0/1000);
    // fcl::Matrix3f me(Re[0][0], Re[0][1], Re[0][2], Re[1][0], Re[1][1], Re[1][2], Re[2][0], Re[2][1], Re[2][2]);
    // fcl::Matrix3f mb(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    // fcl::Matrix3f mm(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    fcl::Quaternionf re(qe.w(), qe.x(), qe.y(), qe.z());
    fcl::Quaternionf rb(1.0, 0.0, 0.0, 0.0);
    fcl::Quaternionf rm(1.0, 0.0, 0.0, 0.0);
    endeffectorObj.setTransform(re, te);
    bodyObj.setTransform(rb, tb);
    mobileObj.setTransform(rm, tm);

    bool collision = false;
    fcl::CollisionRequestf requestType(1, false, 1, false);
    fcl::CollisionResultf collisionResult;
    fcl::collide(&endeffectorObj, &treeObj, requestType, collisionResult);
    collision |= collisionResult.isCollision();
    collisionResult.clear();
    fcl::collide(&endeffectorObj, &bodyObj, requestType, collisionResult);
    collision |= collisionResult.isCollision();
    collisionResult.clear();
    fcl::collide(&endeffectorObj, &mobileObj, requestType, collisionResult);
    collision |= collisionResult.isCollision();

    return !collision;
}

bool GlobalPlanner::isIKValid(vector<float> pos) {
	// Only for arm right
    const float R_UPPER_LIMIT[] = { 90,  180,  20,  180,  130,  180};
    const float R_LOWER_LIMIT[] = {-90, -180, -180, -180, -130, -180};
    
    vector<float> op = {pos[3] * Rad2Angle, pos[4] * Rad2Angle, pos[5] * Rad2Angle, pos[0] * 1000, pos[1] * 1000, pos[2] * 1000};
    ROS_INFO("IK goal: %f %f %f %f %f %f", op[0], op[1], op[2], op[3], op[4], op[5]);
    deque<vector<float>> solutions = CArm->Calculate_IK(op);
    if (solutions.empty()) {
        std::cout << "no solution" << std::endl;
        return false;
    }

	// Check if each motor is out of limit
	float motor_angle = 0;
	vector<int> remove_i;
	bool out_of_limit = false;
	int size = solutions.size();
	for (int i = 0; i < size; i++) {
		out_of_limit = false;
		for (int j = 0; j < 6; j++) {
			motor_angle = solutions[i][j] * Rad2Angle;
			if ((motor_angle >= R_UPPER_LIMIT[j]) || (motor_angle <= R_LOWER_LIMIT[j])) {
				out_of_limit = true;
			}
		}
		if (out_of_limit)
			remove_i.push_back(i);
	}
	std::reverse(remove_i.begin(), remove_i.end());
	for (int i : remove_i) {
		solutions.erase(solutions.begin() + i);
	}
	if (solutions.empty()) {
        std::cout << "out of limit" << std::endl;
        return false;
    }

    return true;
}
ompl::base::OptimizationObjectivePtr GlobalPlanner::getObjWithCostToGo(const ompl::base::SpaceInformationPtr& space_info) {
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(space_info));
    obj->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
    obj->setCostThreshold(ompl::base::Cost(1.5));
    return obj;
}

void GlobalPlanner::SetStart(vector<float> start) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start_state(this->space);
    start_state->setXYZ(start[0] / 1000, start[1] / 1000, start[2] / 1000);
    Eigen::Matrix3f R = Eigen::AngleAxisf(start[5] * Angle2Rad, Eigen::Vector3f::UnitZ())
                      * Eigen::AngleAxisf(start[4] * Angle2Rad, Eigen::Vector3f::UnitY())
                      * Eigen::AngleAxisf(start[3] * Angle2Rad, Eigen::Vector3f::UnitX()).toRotationMatrix();
    Eigen::AngleAxisf aa;
    aa.fromRotationMatrix(R);
    start_state->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(aa.axis().x(), aa.axis().y(), aa.axis().z(), aa.angle());
    pdef->clearStartStates();
    pdef->addStartState(start_state);
}

void GlobalPlanner::SetGoal(vector<float> goal) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal_state(this->space);
    goal_state->setXYZ(goal[0] / 1000, goal[1] / 1000, goal[2] / 1000);
    Eigen::Matrix3f R = Eigen::AngleAxisf(goal[5] * Angle2Rad, Eigen::Vector3f::UnitZ())
                      * Eigen::AngleAxisf(goal[4] * Angle2Rad, Eigen::Vector3f::UnitY())
                      * Eigen::AngleAxisf(goal[3] * Angle2Rad, Eigen::Vector3f::UnitX()).toRotationMatrix();
    Eigen::AngleAxisf aa;
    aa.fromRotationMatrix(R);
    goal_state->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(aa.axis().x(), aa.axis().y(), aa.axis().z(), aa.angle());
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

    std::cout << "space info: ";
    si->printSettings(std::cout);
    std::cout << "problem def: ";
    pdef->print(std::cout);

    ompl::base::PlannerStatus solved = planner->solve(10);

    if (solved) {
        ompl::base::PathPtr Path = pdef->getSolutionPath();
        ompl::geometric::PathGeometric* path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        // path->interpolate();

        // ompl::geometric::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
		// smooth_path = new ompl::geometric::PathGeometric(dynamic_cast<const ompl::geometric::PathGeometric&>(*pdef->getSolutionPath()));
		// pathBSpline->smoothBSpline(*smooth_path, 3);

        std::cout << "Found solution:" << std::endl;
        path->printAsMatrix(std::cout);

        path_states.clear();

        nav_msgs::Path msg;
        path_msg.header.frame_id = "base";
        path_msg.header.stamp = ros::Time::now();
        for (size_t i = 0; i < path->getStateCount(); i++) {
            vector<float> Pstate;
            const ompl::base::SE3StateSpace::StateType* state = path->getState(i)->as<ompl::base::SE3StateSpace::StateType>();
            const ompl::base::RealVectorStateSpace::StateType* pos = state->as<ompl::base::RealVectorStateSpace::StateType>(0);
            const ompl::base::SO3StateSpace::StateType* rot = state->as<ompl::base::SO3StateSpace::StateType>(1);
            geometry_msgs::PoseStamped pose;
            // std::cout << "postion " << i << ": " << pos->values[0] << " " << pos->values[1] << " " << pos->values[2] << std::endl;
            // std::cout << "rotation " << i << ": " << rot->x << " " << rot->y << " " << rot->z << " " << rot->w << std::endl;
            pose.pose.position.x = pos->values[0];
            pose.pose.position.y = pos->values[1];
            pose.pose.position.z = pos->values[2];
            pose.pose.orientation.x = rot->x;
            pose.pose.orientation.y = rot->y;
            pose.pose.orientation.z = rot->z;
            pose.pose.orientation.w = rot->w;
            Pstate.push_back(pos->values[0]);
            Pstate.push_back(pos->values[1]);
            Pstate.push_back(pos->values[2]);
            Pstate.push_back(rot->w);
            Pstate.push_back(rot->x);
            Pstate.push_back(rot->y);
            Pstate.push_back(rot->z);
            path_msg.poses.push_back(pose);
            path_states.push_back(Pstate);

            // Eigen::Quaternionf q(rot->w, rot->x, rot->y, rot->z);
            // Eigen::Matrix3f R = q.toRotationMatrix();
            // Eigen::Vector3f euler = R.eulerAngles(2, 1, 0) * Rad2Angle;

            // airobots_calvin::ArmGoal g;
            // g.pos.clear();
            // g.pos.push_back(1.0);
            // g.pos.push_back(Pstate[0]);
            // g.pos.push_back(Pstate[1]);
            // g.pos.push_back(Pstate[2]);
            // g.pos.push_back(euler[0]);
            // g.pos.push_back(euler[1]);
            // g.pos.push_back(euler[2]);
            // std::cout << "goal: " << g.pos[0] << " " << g.pos[1] << " " << g.pos[2] << " " << g.pos[3] << " " << g.pos[4] << " " << g.pos[5] << " " << g.pos[6] << std::endl;

            // client_armR.pushGoal(g);
        }
        path_pub.publish(msg);
    }
    else {
        cout << "No solution found" << endl;
    }
}

deque<vector<float>> GlobalPlanner::GetPath(void) {
    return path_states;
}

Eigen::Vector3f GlobalPlanner::Q2Euler(const Eigen::Quaternionf& q) {
    Eigen::Vector3f euler;
    euler[0] = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    euler[1] = 2 * atan2(sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z())), sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()))) - M_PI /2;
    euler[2] = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    return euler;
}


void GlobalPlanner::InitialROS(void) {
    pointcloud_sub = n.subscribe("/points2", 1, &OctreeGen::PointCloudCallback, octreeGen);
    motor_sub = n.subscribe("/armR/motorstatus", 1, &OctreeGen::MotorCallback, octreeGen);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(n, "/points2", 1);
    // message_filters::Subscriber<airobots_calvin::MotorStatus> motor_sub(n, "/armR/motorstatus", 1);
    // message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, airobots_calvin::MotorStatus> sync(points_sub, motor_sub, 10);
    // sync.registerCallback(boost::bind(&OctreeGen::PointCloudCallback, octreeGen, _1, _2));
    octree_pub = n.advertise<octomap_msgs::Octomap>("/octomap", 1);
    path_pub = n.advertise<nav_msgs::Path>("/end_path",1);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
    loop_rate = new ros::Rate(30);
    ros_thread = new std::thread(&GlobalPlanner::Ros_spin, this);
}

void GlobalPlanner::Ros_spin(void) {
    while(ros::ok()){
        octree_pub.publish(octreeGen->octree_msg);
        path_pub.publish(path_msg);
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
