#include "LocalPlanner.h"

LocalPlanner* LocalPlanner::inst_ = nullptr;

LocalPlanner* LocalPlanner::GetLocalPlanner(Arm* carm, OctreeGen* octreegen) {
    if (inst_ == nullptr)
        inst_ = new LocalPlanner(carm, octreegen);
    return inst_;
}

LocalPlanner::LocalPlanner(Arm* carm, OctreeGen* octreegen) {
    // GPlanner = GlobalPlanner::GetGlobalPlanner(carm, octreegen);
    // GPlanner->SetStart({300, -300, -300, 0, 90, 0});
    // GPlanner->SetGoal({300, -400, -200, 0, 90, 0});
    // GPlanner->Plan();
    CArm = carm;
    octreeGen = octreegen;
    // Path = GPlanner->GetPath();
    Path.push_back({300, -300, -300, 0, 0.7071068, 0, 0.7071068 });
    Path.push_back({300, -400, -200, 0, 0.7071068, 0, 0.7071068 });
    state << -2.48131605,  0,
                -1.64973371,  0,
                -0.16230579,  0,
                0.01278006,  0,
                1.64867108,  0,
                2.64416641,  0;
    // state << -2.48131605, -1.64973371, -0.16230579,  0.01278006,  1.64867108,  2.64416641,
    //          0, 0, 0, 0, 0, 0;

    // reserve vector size
    joint_angles.reserve(6);
    key_joint_pos.reserve(4);
    ObjRobot.reserve(4);
    obs_pos_list.reserve(6);
    link_pos_list.reserve(6);
    length_list.reserve(6);

    // Create FCL objects
    ObjRobot.clear();
    std::shared_ptr<fcl::CollisionGeometryf> Glink1 (new fcl::Capsulef(0.2 / 2, 0.257));
    std::shared_ptr<fcl::CollisionGeometryf> Glink2 (new fcl::Capsulef(0.2 / 2, 0.32));
    std::shared_ptr<fcl::CollisionGeometryf> Gbody (new fcl::Boxf(80.0/1000, 260.0/1000, 350.0/1000));
    std::shared_ptr<fcl::CollisionGeometryf> Gmobile (new fcl::Boxf(210.0/1000, 440.0/1000, 210.0/1000));
    ObjRobot.push_back(fcl::CollisionObjectf(Glink1));
    ObjRobot.push_back(fcl::CollisionObjectf(Glink2));
    ObjRobot.push_back(fcl::CollisionObjectf(Gbody));
    ObjRobot.push_back(fcl::CollisionObjectf(Gmobile));
    // ObjRobot[0] = fcl::CollisionObjectf(Glink1);
    // ObjRobot[1] = fcl::CollisionObjectf(Glink2);
    // ObjRobot[2] = fcl::CollisionObjectf(Gbody);
    // ObjRobot[3] = fcl::CollisionObjectf(Gmobile);

    fcl::Vector3f tb((0.0)/2/1000, (0.0)/2/1000, -(350.0)/2/1000);
    fcl::Vector3f tm((0.0)/2/1000, (0.0)/2/1000, -(210.0)/2/1000-500.0/1000);
    fcl::Quaternionf rb(1.0, 0.0, 0.0, 0.0);
    fcl::Quaternionf rm(1.0, 0.0, 0.0, 0.0);
    ObjRobot[2].setTransform(rb, tb);
    ObjRobot[3].setTransform(rm, tm);

    // curr = Eigen::Vector3f(CArm->GetCurrentPosition(0), CArm->GetCurrentPosition(1), CArm->GetCurrentPosition(2));
    // last = curr;
    // goal = g;
    dt = 0.1;
    // human_IDlist = {126, 13, 122, 118, 112, 113, 114, 115, 15, 16, 17, 18};
    InitialROS();
    while (octreeGen->ball_pos.norm() == 0)
        ros::Duration(0.1).sleep();
    UpdateFCL();
}

LocalPlanner::~LocalPlanner() {
    StopROS();
    delete this->ros_thread;
    inst_ = nullptr;
}

void LocalPlanner::Run(void) {
    while (!Path.empty())
    {
        vector<float> curr_goal = Path.front();
        Eigen::Vector3f goal_pos = {curr_goal[0], curr_goal[1], curr_goal[2]};
        Eigen::Vector4f goal_ori = {curr_goal[3], curr_goal[4], curr_goal[5], curr_goal[6]};
        if (octreeGen->update_flag) {
            UpdateFCL();
            octreeGen->update_flag = false;
            float error = Move(goal_pos, goal_ori);
            std::cout << "q = " << state(0, 0) << ", " << state(1, 0) << ", " << state(2, 0) << ", " << state(3, 0) << ", " << state(4, 0) << ", " << state(5, 0) << std::endl;
            // airobots_calvin::ArmGoal g;
            // g.pos.clear();
            // g.pos.push_back(2.0);
            // g.pos.push_back(state(0, 0));
            // g.pos.push_back(state(1, 0));
            // g.pos.push_back(state(2, 0));
            // g.pos.push_back(state(3, 0));
            // g.pos.push_back(state(4, 0));
            // g.pos.push_back(state(5, 0);

            // client_armR.pushGoal(g);

            ros::Duration(0.1).sleep();

            if (error < 0.1)
                Path.pop_front();
        }
    }
}

void LocalPlanner::UpdateFCL(void) {
    // curr_octree = octreeGen->curr_octree;
    // fcl::OcTreef* tree = new fcl::OcTreef(std::shared_ptr<const octomap::OcTree>(curr_octree));
    // curr_tree = std::shared_ptr<fcl::CollisionGeometryf>(tree);
    ball_pos = octreeGen->ball_pos;
    curr_tree = std::shared_ptr<fcl::CollisionGeometryf>(new fcl::Spheref(0.05));
    UpdateJoints();
    UpdateDis();
}

void LocalPlanner::UpdateJoints(void) {
    joint_angles.clear();
    const int FIRST_HAND_ID_ = 23;
    // ArmRinit = np.array([-2.48131605, -1.64973371, -0.16230579,  0.01278006,  1.64867108,  2.64416641])
    joint_angles = {state(0, 0), state(1, 0), state(2, 0), state(3, 0), state(4, 0), state(5, 0)};
    key_joint_pos = octreeGen->getJointPosition(joint_angles);
    // joint_angles = octreeGen->getJointAngles();
    // key_joint_pos = octreeGen->getKeyJointPos();
    // for (int i = 0; i < 6; i++)
    //     joint_angles.push_back(CArm->GetMotor_PresentAngle(FIRST_HAND_ID_ + 0));
    // key_joint_pos = GPlanner->octreeGen->getJointPosition(joint_angles);
    UpdateArmFCL();
}

void LocalPlanner::UpdateArmFCL(void) {
    fcl::Vector3f t1((key_joint_pos[1][0] + key_joint_pos[2][0])/2, (key_joint_pos[1][1] + key_joint_pos[2][1])/2, (key_joint_pos[1][2] + key_joint_pos[2][2])/2);
    fcl::Vector3f t2((key_joint_pos[2][0] + key_joint_pos[3][0])/2, (key_joint_pos[2][1] + key_joint_pos[3][1])/2, (key_joint_pos[2][2] + key_joint_pos[3][2])/2);
    Eigen::Quaternionf q1 = Vec2Q(key_joint_pos[1] - key_joint_pos[2]);
    Eigen::Quaternionf q2 = Vec2Q(key_joint_pos[2] - key_joint_pos[3]);
    fcl::Quaternionf r1(q1.w(), q1.x(), q1.y(), q1.z());
    fcl::Quaternionf r2(q2.w(), q2.x(), q2.y(), q2.z());
    ObjRobot[0].setTransform(r1, t1);
    ObjRobot[1].setTransform(r2, t2);
}

void LocalPlanner::UpdateDis(void) {
    obs_pos_list.clear();
    link_pos_list.clear();
    length_list.clear();

    fcl::CollisionObjectf tree = fcl::CollisionObjectf(curr_tree);
    CollsionPair(0, ObjRobot[0], tree);
    CollsionPair(1, ObjRobot[0], ObjRobot[2]);
    CollsionPair(2, ObjRobot[0], ObjRobot[3]);
    CollsionPair(3, ObjRobot[1], tree);
    CollsionPair(4, ObjRobot[1], ObjRobot[2]);
    CollsionPair(5, ObjRobot[1], ObjRobot[3]);
}

void LocalPlanner::CollsionPair(const int& idx, fcl::CollisionObjectf& obj1, fcl::CollisionObjectf& obj2) {
    const int link = (idx < 3) ? 1 : 2;
    fcl::DistanceRequestf request;
    fcl::DistanceResultf result;
    fcl::distance(&obj1, &obj2, request, result);
    Eigen::Vector3f surface = Eigen::Vector3f(result.nearest_points[1][0], result.nearest_points[1][1], result.nearest_points[1][2]);
    float l = (key_joint_pos[link + 1] - key_joint_pos[link]).dot(surface - key_joint_pos[link]);
    length_list.push_back(l);
    // Eigen::Vector3f control_pos = key_joint_pos[link] + l * (key_joint_pos[link + 1] - key_joint_pos[link]) / (key_joint_pos[link + 1] - key_joint_pos[link]).norm();
    link_pos_list.push_back(surface);
    obs_pos_list.push_back(Eigen::Vector3f(result.nearest_points[0][0], result.nearest_points[0][1], result.nearest_points[0][2]));
}

float LocalPlanner::Move(const Eigen::Vector3f& goal_pos, const Eigen::Vector4f& goal_ori) {
    std::shared_ptr<RMPRoot> Root = getRMP(goal_pos, goal_ori);
    Eigen::Matrix<float, 6, 2> state_dot = Dynamics(state, Root);
    Eigen::Matrix<float, 6, 2> new_state = state + state_dot * dt;
    const float max_q_dot = 0.2;
    for (int i = 0; i < 6; i++) {
        if (new_state(i, 1) > max_q_dot)
            new_state(i, 1) = max_q_dot;
        if (new_state(i, 1) < -max_q_dot)
            new_state(i, 1) = -max_q_dot;
    }
    state = new_state;
    float error = 0.0;
    for (auto& Task : Root->children) {
        for (auto& N : Task->children) {
            if (N->isLeaf) {
                error += N->get_error();
            }
        }
    }
    std::cout << "error = " << error << std::endl;
    return error;
}

Eigen::Matrix<float, 6, 2> LocalPlanner::Dynamics(Eigen::Matrix<float, 6, 2>& s, std::shared_ptr<RMPRoot> root) {
    Eigen::MatrixXf x(6, 1);
    x = s.block(0, 0, 6, 1);
    Eigen::MatrixXf x_dot(6, 1);
    x_dot = s.block(0, 1, 6, 1);
    std::cout << "x = " << x << std::endl;
    std::cout << "x_dot = " << x_dot << std::endl;
    Eigen::MatrixXf result = root->solve(x, x_dot);
    std::cout << "result = " << result << std::endl;
    Eigen::Matrix<float, 6, 2> state_dot;
    state_dot << x_dot.block(0, 0, 6, 1), result.block(0, 0, 6, 1);
    return state_dot;
}

std::shared_ptr<RMPRoot> LocalPlanner::getRMP(const Eigen::Vector3f& goal_pos, const Eigen::Vector4f& goal_ori) {
    Eigen::MatrixXf Pgoal(3, 1);
    Pgoal = goal_pos;
    Eigen::MatrixXf Qgoal(4, 1);
    Qgoal = goal_ori;

    // Goal Reaching
    std::shared_ptr<RMPRoot> Root(new RMPRoot("Root"));
    std::shared_ptr<TransitionTaskSpace> eeT(new TransitionTaskSpace("eeT", Root, 2, 0.32));
    eeT->parent->add_child(eeT);
    std::shared_ptr<GoalAttractorUniT> GT(new GoalAttractorUniT("GT", eeT, Pgoal));
    GT->parent->add_child(GT);
    
    std::shared_ptr<RotationTaskSpace> eeR(new RotationTaskSpace("eeR", Root, 2, 0.32));
    eeR->parent->add_child(eeR);
    std::shared_ptr<GoalAttractorUniR> GR(new GoalAttractorUniR("GR", eeR, Qgoal));
    GR->parent->add_child(GR);

    // link1
    std::shared_ptr<TransitionTaskSpace> link1C(new TransitionTaskSpace("link1C", Root, 1, length_list[0]));
    link1C->parent->add_child(link1C);
    std::shared_ptr<TransitionArbitraryPoint> link1CC(new TransitionArbitraryPoint("link1CC", link1C, link_pos_list[0]));
    link1CC->parent->add_child(link1CC);
    std::shared_ptr<CollisionAvoidance> C1(new CollisionAvoidance("C1", link1CC, obs_pos_list[0], 0.05f));
    C1->parent->add_child(C1);

    std::shared_ptr<TransitionTaskSpace> link1B(new TransitionTaskSpace("link1B", Root, 1, length_list[1]));
    link1B->parent->add_child(link1B);
    std::shared_ptr<TransitionArbitraryPoint> link1CB(new TransitionArbitraryPoint("link1CB", link1B, link_pos_list[1]));
    link1CB->parent->add_child(link1CB);
    std::shared_ptr<CollisionAvoidance> B1(new CollisionAvoidance("B1", link1CB, obs_pos_list[1], 0.05f));
    B1->parent->add_child(B1);

    std::shared_ptr<TransitionTaskSpace> link1M(new TransitionTaskSpace("link1M", Root, 1, length_list[2]));
    link1M->parent->add_child(link1M);
    std::shared_ptr<TransitionArbitraryPoint> link1CM(new TransitionArbitraryPoint("link1CM", link1M, link_pos_list[2]));
    link1CM->parent->add_child(link1CM);
    std::shared_ptr<CollisionAvoidance> M1(new CollisionAvoidance("M1", link1CM, obs_pos_list[2], 0.05f));
    M1->parent->add_child(M1);

    // link2
    std::shared_ptr<TransitionTaskSpace> link2C(new TransitionTaskSpace("link2C", Root, 2, length_list[3]));
    link2C->parent->add_child(link2C);
    std::shared_ptr<TransitionArbitraryPoint> link2CC(new TransitionArbitraryPoint("link2CC", link2C, link_pos_list[3]));
    link2CC->parent->add_child(link2CC);
    std::shared_ptr<CollisionAvoidance> C2(new CollisionAvoidance("C2", link2CC, obs_pos_list[3], 0.05f));
    C2->parent->add_child(C2);

    std::shared_ptr<TransitionTaskSpace> link2B(new TransitionTaskSpace("link2B", Root, 2, length_list[4]));
    link2B->parent->add_child(link2B);
    std::shared_ptr<TransitionArbitraryPoint> link2CB(new TransitionArbitraryPoint("link2CB", link2B, link_pos_list[4]));
    link2CB->parent->add_child(link2CB);
    std::shared_ptr<CollisionAvoidance> B2(new CollisionAvoidance("B2", link2CB, obs_pos_list[4], 0.05f));
    B2->parent->add_child(B2);

    std::shared_ptr<TransitionTaskSpace> link2M(new TransitionTaskSpace("link2M", Root, 2, length_list[5]));
    link2M->parent->add_child(link2M);
    std::shared_ptr<TransitionArbitraryPoint> link2CM(new TransitionArbitraryPoint("link2CM", link2M, link_pos_list[5]));
    link2CM->parent->add_child(link2CM);
    std::shared_ptr<CollisionAvoidance> M2(new CollisionAvoidance("M2", link2CM, obs_pos_list[5], 0.05f));
    M2->parent->add_child(M2);
    
    return Root;
}

void LocalPlanner::UpdateState(void) {
    // // t1 = ros::Time::now();
    // // if (t2 != 0)
    // //     dt = t1 - t2;
    // // t2 = t1;
    // std::shared_ptr<fcl::CollisionGeometryf> end_shape (new fcl::Spheref(25));
    // fcl::CollisionObjectf end = fcl::CollisionObjectf(end_shape);
    // end.setTranslation(fcl::Vector3f(curr[0], curr[1], curr[2]));
    // CreateFCL();
    // fcl::DistanceRequestf request;
    // fcl::DistanceResultf result;
    // last_obs.clear();
    // last_obs = curr_obs;
    // curr_obs.clear();
    // for (auto& ob : human_body) {
    //     fcl::distance(&end, &ob, request, result);
    //     curr_obs.push_back(Eigen::Vector3f(result.nearest_points[1][0], result.nearest_points[1][1], result.nearest_points[1][2]));
    // }
    // APF_acc();
    // curr = curr + ((curr - last) / dt + acc * dt);
    // // TODO: set curr
    // // CArm->TrajectoryPlanning(0, 90, 0, curr[0], curr[1], curr[2]);
    // last = curr;
}
void LocalPlanner::APF_acc(void) {
    // Goal approach
    Eigen::Vector3f x_goal = goal - curr;
    Eigen::Vector3f v_goal = (x_goal - (goal - last)) / dt;
    float acc_mag = Attractor(x_goal.norm(), v_goal.norm());
    acc = acc_mag / x_goal.norm() * x_goal;

    // Collision avoidance
    Eigen::Vector3f x_avoid, v_avoid;
    for (int i = 0; i < curr_obs.size(); i++) {
        x_avoid = curr_obs[i] - curr;
        v_avoid = (x_avoid - (last_obs[i] - last)) / dt;
        acc_mag = Distractor(x_avoid.norm(), v_avoid.norm());
        acc += acc_mag / x_avoid.norm() * x_avoid;
    }
}
float LocalPlanner::Attractor(float x, float x_dot) {
    const float w_u = 10, w_l = 1, sigma = 1, alpha = 1, eta = 2, gain = 1, tol = 0.005;
    float beta, w, s, g, grad_Phi, Bx_dot, grad_w, x_dot_norm, xi, M, f;

    beta = exp(- x * x / 2 / (sigma * sigma));
    w = (w_u - w_l) * beta + w_l;
    s = (1 - exp(-2 * alpha * x)) / (1 + exp(-2 * alpha * x));
    if (x > tol) {
        grad_Phi = s * w;
    } else {
        grad_Phi = 0;
    }
    Bx_dot = eta * w * x_dot;
    grad_w = - beta * (w_u - w_l) / (sigma * sigma) * x;
    xi = 0.5 * x_dot * x_dot * grad_w;
    M = w;
    f = -grad_Phi - Bx_dot - xi;

    float accA = f / M;
    return accA;
}

float LocalPlanner::Distractor(float x, float x_dot) {
    const float epsilon = 0.2, alpha = 1e-5, eta = 0;
    float w, grad_w, u, g, grad_u, grad_Phi, xi, M, Bx_dot, f;

    if (x < 0) {
        w = 1e10;
        grad_w = 0;
    } else {
        w = (1.0 / pow(x, 4));
        grad_w = (-4.0 / pow(x, 5));
    }
    u = epsilon + fmin(0.0, x_dot) * x_dot;
    g = w * u;
    grad_u = 2 * fmin(0.0, x_dot);
    grad_Phi = alpha * w * grad_w;
    xi = 0.5 * pow(x_dot, 2) * u * grad_w;
    M = g + 0.5 * x_dot * w * grad_u;
    M = fmin(fmax(M, -1e5), 1e5);
    Bx_dot = eta * g * x_dot;
    f = -grad_Phi - xi - Bx_dot;
    f = fmin(fmax(f, -1e10), 1e10);
    
    float accD = f / M;
    return accD;
}

void LocalPlanner::CreateFCL(void) {
    // vector<float> FCLsize = {125, (human[4]-human[8]).norm()/2, ((human[4]+human[8])/2-(human[2]+human[3])/2).norm(), 50, 100,
    //                          (human[4]-human[5]).norm(), (human[5]-human[6]).norm(), 
    //                          (human[8]-human[9]).norm(), (human[9]-human[10]).norm()};
    // vector<std::shared_ptr<fcl::CollisionGeometryf>> human_shape;
    // // human_shape[0] = std::make_shared<fcl::CollisionGeometryf>(fcl::Spheref(FCLsize[0]));
    // human_shape[0] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Spheref(FCLsize[0]));
    // human_shape[1] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Cylinderf(FCLsize[1], FCLsize[2]));
    // human_shape[2] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Cylinderf(FCLsize[3], FCLsize[5]));
    // human_shape[3] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Cylinderf(FCLsize[3], FCLsize[6]));
    // human_shape[4] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Spheref(FCLsize[4]));
    // human_shape[5] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Cylinderf(FCLsize[3], FCLsize[7]));
    // human_shape[6] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Cylinderf(FCLsize[3], FCLsize[8]));
    // human_shape[7] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Spheref(FCLsize[4]));
    // vector<Eigen::Vector3f> eigen_T = {2*human[0]-human[1], (human[4]+human[8])/2, human[4], human[5], human[7], human[8], human[9], human[11]};
    // vector<Eigen::Vector3f> eigen_q = {((human[2]+human[3])/2-(human[4]+human[8])/2)/((human[2]+human[3])/2-(human[4]+human[8])/2).norm(), 
    //                                    (human[5]-human[4])/(human[5]-human[4]).norm(), (human[6]-human[5])/(human[6]-human[5]).norm(), 
    //                                    (human[9]-human[8])/(human[9]-human[8]).norm(), (human[10]-human[9])/(human[10]-human[9]).norm()};
    // vector<fcl::Vector3f> FCL_T;
    // for (auto& T : eigen_T) {
    //     FCL_T.push_back(fcl::Vector3f(T[0], T[1], T[2]));
    // }
    // vector<fcl::Quaternionf> FCL_q;
    // for (auto& q : eigen_q) {
    //     fcl::Quaternionf tmp_q;
    //     FCL_q.push_back(fcl::Quaternionf(0, q[0], q[1], q[2]));
    // }
    // human_body.clear();
    // for (size_t i = 0; i < human_shape.size(); i++) {
    //     human_body[i] = fcl::CollisionObjectf(human_shape[i]);
    // }
    // human_body[0].setTranslation(FCL_T[0]);
    // human_body[1].setTransform(FCL_q[0], FCL_T[1]);
    // human_body[2].setTransform(FCL_q[1], FCL_T[2]);
    // human_body[3].setTransform(FCL_q[2], FCL_T[3]);
    // human_body[4].setTranslation(FCL_T[4]);
    // human_body[5].setTransform(FCL_q[3], FCL_T[5]);
    // human_body[6].setTransform(FCL_q[4], FCL_T[6]);
    // human_body[7].setTranslation(FCL_T[7]);
}
void LocalPlanner::HumanCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    // human.clear();
    // human.resize(12, Eigen::Vector3f::Zero());
    // Eigen::Vector3f p;
    // for (auto& marker : msg->markers) {
    //     for (int j = 0; j < 12; j++) {
    //         if (human_IDlist[j] == marker.id)
    //             p = {static_cast<float>(marker.pose.position.x), static_cast<float>(marker.pose.position.y), static_cast<float>(marker.pose.position.z)};
    //         human[j] = p / 1000;
    //     }
    // }
    // UpdateState();
}

Eigen::Quaternionf LocalPlanner::Vec2Q(const Eigen::Vector3f& lz) {
    Eigen::Vector3f z = {0, 0, 1};
    Eigen::Vector3f unit_lz = lz / lz.norm();
    Eigen::Vector3f v = z.cross(unit_lz);
    // if (v[0] == 0. && v[1] == 0. && v[2] == 0.)
    //     rot.setIdentity();
    Eigen::Matrix3f v_skew;
    v_skew << 0, -v[2], v[1],
              v[2], 0, -v[0],
              -v[1], v[0], 0;
    float s = v.norm();
    float c = z.dot(unit_lz);
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R = I + v_skew + v_skew * v_skew / (1 + c);
    Eigen::Quaternionf q(R);
    return q;
}

void LocalPlanner::InitialROS(void) {
    // human_sub = n.subscribe("/body_tracking_data", 1, &LocalPlanner::HumanCallback, this);
    std::cout << "Initializing ROS..." << std::endl;
    ball_sub = n.subscribe("/ball", 1, &OctreeGen::BallCallback, octreeGen);
    loop_rate = new ros::Rate(30);
    ros_thread = new std::thread(&LocalPlanner::Ros_spin, this);
    std::cout << "ROS initialized" << std::endl;
}
void LocalPlanner::Ros_spin(void) {
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate->sleep();
    }
    // ros::spin();
}
void LocalPlanner::StopROS(void) {
    human_sub.shutdown();
    ball_sub.shutdown();
    delete loop_rate;
}