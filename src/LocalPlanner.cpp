#include "LocalPlanner.h"

LocalPlanner* LocalPlanner::inst_ = nullptr;

LocalPlanner* LocalPlanner::GetLocalPlanner(bool* t, Arm* carm, OctreeGen* octreegen) {
    if (inst_ == nullptr)
        inst_ = new LocalPlanner(t, carm, octreegen);
    return inst_;
}

LocalPlanner::LocalPlanner(bool* t, Arm* carm, OctreeGen* octreegen) : terminated(t)/*,  client_armR("armR") */{
    // GPlanner = GlobalPlanner::GetGlobalPlanner(carm, octreegen);
    // GPlanner->SetStart({300, -300, -300, 0, 90, 0});
    // GPlanner->SetGoal({300, -400, -200, 0, 90, 0});
    // GPlanner->Plan();
    CArm = carm;
    octreeGen = octreegen;
    // client_armR.run();

    // Path = GPlanner->GetPath();
    // Path.push_back({0.3, -0.3, -0.3, 0.7071068, 0, 0.7071068, 0 });
    // Path.push_back({0.3, -0.4, -0.2, 0.7071068, 0, 0.7071068, 0 });
    // Path.push_back({-0.36919227, 1.65581407, -0.41166997, 3.10750463, 1.6486961, 3.09778644});
    // Path.push_back({-0.36919227, -1.48577858, -2.72992269, -0.03408802, 1.6486961, 3.09778644});
    
    Path.push_back({0.02106348, -1.04055644, -2.74725864, -0.22153266,  2.05660777, 2.671088});
    Path.push_back({0.16986938, -1.89871067, -2.88207129,  0.08707968,  1.25425242, 2.69829482});
    // Path.push_back({0.23360576, -1.77663517, -2.94344571,  0.04107898,  1.36904126, 2.7056813});
    Path.push_back({-0.73994928, -1.04055644, -2.74725864, -0.22153266,  2.05660777, -2.85108455});
    // Path.push_back({-2.23536091, -2.10103622, -0.39433402,  0.22153266,  2.05660777, 2.68480209});

    int init_q = 1;
    // std::cout << "Enter initial q: ";
    // std::cin >> init_q;
    if (init_q == 0) {
        state << 0, 0,
                0, 0,
                0, 0,
                0, 0,
                0, 0,
                0, 0;
    } else if (init_q == 1) {
        state << 0, 0,
                0, 0,
                -0.18, 0,
                0, 0,
                0, 0,
                0, 0;
    } else if (init_q == 2) {
        state << -0.36919227,  0,
                1.65581407,  0,
                -0.41166997,  0,
                3.10750463,  0,
                1.6486961,   0,
                3.09778644,  0;
    } else {
        state << -2.48131605,  0,
                    -1.64973371,  0,
                    -0.16230579,  0,
                    0.01278006,  0,
                    1.64867108,  0,
                    2.64416641,  0;
    }
    // std::cout << "Enter max_iter: ";
    // std::cin >> max_iter;
    max_iter = 5000;
    // state << -2.48131605, -1.64973371, -0.16230579,  0.01278006,  1.64867108,  2.64416641,
    //          0, 0, 0, 0, 0, 0;

    // reserve vector size
    joint_angles.reserve(6);
    key_joint_pos.reserve(4);
    ObjRobot.reserve(9);
    obs_pos_list.reserve(10);
    link_pos_list.reserve(10);
    length_list.reserve(10);

    // Create FCL objects
    ObjRobot.clear();
    std::shared_ptr<fcl::CollisionGeometryf> Glink1 = std::make_shared<fcl::Capsulef>(0.05, 0.24);
    std::shared_ptr<fcl::CollisionGeometryf> Glink2 = std::make_shared<fcl::Capsulef>(0.08, 0.24);
    // std::shared_ptr<fcl::CollisionGeometryf> Glink1 = std::make_shared<fcl::Cylinderf>(0.08, 0.257);
    // std::shared_ptr<fcl::CollisionGeometryf> Glink2 = std::make_shared<fcl::Cylinderf>(0.08, 0.32);
    std::shared_ptr<fcl::CollisionGeometryf> Gbody = std::make_shared<fcl::Boxf>(0.08, 0.26, 0.6);
    std::shared_ptr<fcl::CollisionGeometryf> Gmobile = std::make_shared<fcl::Boxf>(0.44, 0.44, 0.44);
    std::shared_ptr<fcl::CollisionGeometryf> Gball = std::make_shared<fcl::Boxf>(0.12, 0.065, 0.225);
    // std::shared_ptr<fcl::CollisionGeometryf> Gball = std::make_shared<fcl::Spheref>(0.05);
    std::shared_ptr<fcl::CollisionGeometryf> Gjoint = std::make_shared<fcl::Spheref>(0.08);
    std::shared_ptr<fcl::CollisionGeometryf> Gtable = std::make_shared<fcl::Boxf>(0.8, 2, 0.6);
    // std::shared_ptr<fcl::CollisionGeometryf> Gball2 = std::make_shared<fcl::Spheref>(0.05);
    std::shared_ptr<fcl::CollisionGeometryf> Gbox = std::make_shared<fcl::Boxf>(0.1, 0.1, 0.085);
    std::shared_ptr<fcl::CollisionGeometryf> Gcube = std::make_shared<fcl::Boxf>(0.045, 0.045, 0.045);
    ObjRobot.push_back(fcl::CollisionObjectf(Glink1));
    ObjRobot.push_back(fcl::CollisionObjectf(Glink2));
    ObjRobot.push_back(fcl::CollisionObjectf(Gbody));
    ObjRobot.push_back(fcl::CollisionObjectf(Gmobile));
    ObjRobot.push_back(fcl::CollisionObjectf(Gball));
    ObjRobot.push_back(fcl::CollisionObjectf(Gjoint));
    ObjRobot.push_back(fcl::CollisionObjectf(Gtable));
    // ObjRobot.push_back(fcl::CollisionObjectf(Gball2));
    ObjRobot.push_back(fcl::CollisionObjectf(Gbox));
    ObjRobot.push_back(fcl::CollisionObjectf(Gcube));
    

    fcl::Vector3f tb(0, 0, -0.165);
    fcl::Vector3f tm(-0.115, 0, -0.685);
    fcl::Vector3f tt(0.55, 0, -0.7865);
    // fcl::Vector3f tbox(0.3, -0.2, -0.2815);
    // fcl::Vector3f tbox(0.3, -0.2, -0.3365);
    fcl::Vector3f tbox(0.3, -0.2, -0.344);
    // fcl::Vector3f tcube(0.2, -0.4, -0.3715);
    fcl::Vector3f tcube(0.2, -0.4, -0.364);
    fcl::Quaternionf rb(1.0, 0.0, 0.0, 0.0);
    fcl::Quaternionf rm(1.0, 0.0, 0.0, 0.0);
    fcl::Quaternionf rt(1.0, 0.0, 0.0, 0.0);
    fcl::Quaternionf rbox(1.0, 0.0, 0.0, 0.0);
    fcl::Quaternionf rcube(1.0, 0.0, 0.0, 0.0);
    ObjRobot[2].setTransform(rb, tb);
    ObjRobot[3].setTransform(rm, tm);
    ObjRobot[6].setTransform(rt, tt);
    ObjRobot[7].setTransform(rbox, tbox);
    ObjRobot[8].setTransform(rcube, tcube);

    // curr = Eigen::Vector3f(CArm->GetCurrentPosition(0), CArm->GetCurrentPosition(1), CArm->GetCurrentPosition(2));
    // last = curr;
    // goal = g;
    dt = 0.1;
    // human_IDlist = {126, 13, 122, 118, 112, 113, 114, 115, 15, 16, 17, 18};
    InitialROS();
    while (octreeGen->ball_pos.norm() == 0)
        ros::Duration(0.1).sleep();
    ball_pos2 = {0.33, -0.5, -0.28};
    UpdateFCL();
}

LocalPlanner::~LocalPlanner() {
    StopROS();
    inst_ = nullptr;
}

void LocalPlanner::Run(void) {
    int count = 0;
    while (!Path.empty() && count < max_iter)
    {
        vector<float> curr_goal = Path.front();
        // Eigen::Vector3f goal_pos = {curr_goal[0], curr_goal[1], curr_goal[2]};
        // Eigen::Vector4f goal_ori = {curr_goal[3], curr_goal[4], curr_goal[5], curr_goal[6]};
        if (octreeGen->update_flag) {
            UpdateFCL();
            octreeGen->update_flag = false;
            float error = Move(curr_goal);
            std::cout << "q = " << state(0, 0) << ", " << state(1, 0) << ", " << state(2, 0) << ", " << state(3, 0) << ", " << state(4, 0) << ", " << state(5, 0) << std::endl;
            PubJoints();
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

            // ros::Duration(0.1).sleep();

            if (error < 0.005)
                Path.pop_front();
            count++;
        }
    }
    float cubeZ = ObjRobot[8].getTranslation().z();
    while (cubeZ > -0.364) {
        cubeZ -= 0.00004;
        ObjRobot[8].setTranslation(fcl::Vector3f(0.3, -0.2, cubeZ));
        PubMarkers();
        ros::Duration(1 / 30).sleep();
    }
    std::cout << "Run finished" << std::endl;
}

void LocalPlanner::UpdateFCL(void) {
    // curr_octree = octreeGen->curr_octree;
    // fcl::OcTreef* tree = new fcl::OcTreef(std::shared_ptr<const octomap::OcTree>(curr_octree));
    // curr_tree = std::shared_ptr<fcl::CollisionGeometryf>(tree);
    ball_pos = octreeGen->ball_pos;
    // ball_pos2 = ball_pos2 + Eigen::Vector3f(-0.001, 0.0, 0.0);
    // curr_tree = std::make_shared<fcl::Spheref>(0.05);
    // curr_tree->setTranslation(fcl::Vector3f(ball_pos[0], ball_pos[1], ball_pos[2]));

    UpdateJoints();
    UpdateDis();
}

void LocalPlanner::UpdateJoints(void) {
    joint_angles.clear();
    const int FIRST_HAND_ID_ = 23;
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
    ObjRobot[4].setTranslation(fcl::Vector3f(ball_pos[0], ball_pos[1], ball_pos[2]));
    ObjRobot[5].setTranslation(fcl::Vector3f(key_joint_pos[2][0], key_joint_pos[2][1], key_joint_pos[2][2]));
    if (Path.size() <= 1) {
        fcl::Vector3f tcube(key_joint_pos[3][0], key_joint_pos[3][1], key_joint_pos[3][2] + 0.0285);
        ObjRobot[8].setTransform(r2, tcube);
        // ObjRobot[8].setTranslation(fcl::Vector3f(key_joint_pos[3][0], key_joint_pos[3][1], key_joint_pos[3][2]));
        // ObjRobot[8].setQuatRotation(r2);
    }
    // ObjRobot[7].setTranslation(fcl::Vector3f(ball_pos2[0], ball_pos2[1], ball_pos2[2]));
    PubMarkers();
}

void LocalPlanner::PubMarkers(void) {
    vector<int> shape = {3, 3, 1, 1};
    vector<Eigen::Vector3f> scale = {Eigen::Vector3f(0.10, 0.10, 0.24), Eigen::Vector3f(0.16, 0.16, 0.24), Eigen::Vector3f(0.08, 0.26, 0.6), Eigen::Vector3f(0.44, 0.44, 0.44)};
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(9);
    for (int i = 0; i < 4; i++) {
        marker_array.markers[i].header.frame_id = "base";
        marker_array.markers[i].header.stamp = ros::Time::now();
        marker_array.markers[i].ns = "basic_shapes";
        marker_array.markers[i].id = i;
        marker_array.markers[i].type = shape[i];
        marker_array.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array.markers[i].pose.position.x = ObjRobot[i].getTranslation().x();
        marker_array.markers[i].pose.position.y = ObjRobot[i].getTranslation().y();
        marker_array.markers[i].pose.position.z = ObjRobot[i].getTranslation().z();
        marker_array.markers[i].pose.orientation.x = ObjRobot[i].getQuatRotation().x();
        marker_array.markers[i].pose.orientation.y = ObjRobot[i].getQuatRotation().y();
        marker_array.markers[i].pose.orientation.z = ObjRobot[i].getQuatRotation().z();
        marker_array.markers[i].pose.orientation.w = ObjRobot[i].getQuatRotation().w();
        marker_array.markers[i].scale.x = scale[i][0];
        marker_array.markers[i].scale.y = scale[i][1];
        marker_array.markers[i].scale.z = scale[i][2];
        marker_array.markers[i].color.a = 0.4;
        marker_array.markers[i].color.r = 0.0;
        marker_array.markers[i].color.g = 0.7;
        marker_array.markers[i].color.b = 0.7;
    }

    marker_array.markers[4].header.frame_id = "base";
    marker_array.markers[4].header.stamp = ros::Time::now();
    marker_array.markers[4].ns = "basic_shapes";
    marker_array.markers[4].id = 4;
    // marker_array.markers[4].type = visualization_msgs::Marker::SPHERE;
    marker_array.markers[4].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[4].action = visualization_msgs::Marker::ADD;
    marker_array.markers[4].pose.position.x = ObjRobot[4].getTranslation().x();
    marker_array.markers[4].pose.position.y = ObjRobot[4].getTranslation().y();
    marker_array.markers[4].pose.position.z = ObjRobot[4].getTranslation().z();
    marker_array.markers[4].pose.orientation.x = 0.0;
    marker_array.markers[4].pose.orientation.y = 0.0;
    marker_array.markers[4].pose.orientation.z = 0.0;
    marker_array.markers[4].pose.orientation.w = 1.0;
    marker_array.markers[4].scale.x = 0.12;
    marker_array.markers[4].scale.y = 0.065;
    marker_array.markers[4].scale.z = 0.225;
    marker_array.markers[4].color.a = 0.7;
    marker_array.markers[4].color.r = 0.7;
    marker_array.markers[4].color.g = 0.7;
    marker_array.markers[4].color.b = 0.0;

    marker_array.markers[5].header.frame_id = "base";
    marker_array.markers[5].header.stamp = ros::Time::now();
    marker_array.markers[5].ns = "basic_shapes";
    marker_array.markers[5].id = 5;
    marker_array.markers[5].type = visualization_msgs::Marker::SPHERE;
    marker_array.markers[5].action = visualization_msgs::Marker::ADD;
    marker_array.markers[5].pose.position.x = ObjRobot[5].getTranslation().x();
    marker_array.markers[5].pose.position.y = ObjRobot[5].getTranslation().y();
    marker_array.markers[5].pose.position.z = ObjRobot[5].getTranslation().z();
    marker_array.markers[5].pose.orientation.x = 0.0;
    marker_array.markers[5].pose.orientation.y = 0.0;
    marker_array.markers[5].pose.orientation.z = 0.0;
    marker_array.markers[5].pose.orientation.w = 1.0;
    marker_array.markers[5].scale.x = 0.16;
    marker_array.markers[5].scale.y = 0.16;
    marker_array.markers[5].scale.z = 0.16;
    marker_array.markers[5].color.a = 0.4;
    marker_array.markers[5].color.r = 0.0;
    marker_array.markers[5].color.g = 0.7;
    marker_array.markers[5].color.b = 0.7;

    marker_array.markers[6].header.frame_id = "base";
    marker_array.markers[6].header.stamp = ros::Time::now();
    marker_array.markers[6].ns = "basic_shapes";
    marker_array.markers[6].id = 6;
    marker_array.markers[6].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[6].action = visualization_msgs::Marker::ADD;
    marker_array.markers[6].pose.position.x = ObjRobot[6].getTranslation().x();
    marker_array.markers[6].pose.position.y = ObjRobot[6].getTranslation().y();
    marker_array.markers[6].pose.position.z = ObjRobot[6].getTranslation().z();
    marker_array.markers[6].pose.orientation.x = 0;
    marker_array.markers[6].pose.orientation.y = 0;
    marker_array.markers[6].pose.orientation.z = 0;
    marker_array.markers[6].pose.orientation.w = 1;
    marker_array.markers[6].scale.x = 0.8;
    marker_array.markers[6].scale.y = 2;
    marker_array.markers[6].scale.z = 0.8;
    marker_array.markers[6].color.a = 0.4;
    marker_array.markers[6].color.r = 1.0;
    marker_array.markers[6].color.g = 0.6;
    marker_array.markers[6].color.b = 0.0;

    marker_array.markers[7].header.frame_id = "base";
    marker_array.markers[7].header.stamp = ros::Time::now();
    marker_array.markers[7].ns = "basic_shapes";
    marker_array.markers[7].id = 7;
    marker_array.markers[7].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[7].action = visualization_msgs::Marker::ADD;
    marker_array.markers[7].pose.position.x = ObjRobot[7].getTranslation().x();
    marker_array.markers[7].pose.position.y = ObjRobot[7].getTranslation().y();
    marker_array.markers[7].pose.position.z = ObjRobot[7].getTranslation().z();
    marker_array.markers[7].pose.orientation.x = 0.0;
    marker_array.markers[7].pose.orientation.y = 0.0;
    marker_array.markers[7].pose.orientation.z = 0.0;
    marker_array.markers[7].pose.orientation.w = 1.0;
    marker_array.markers[7].scale.x = 0.1;
    marker_array.markers[7].scale.y = 0.1;
    marker_array.markers[7].scale.z = 0.085;
    marker_array.markers[7].color.a = 0.4;
    marker_array.markers[7].color.r = 1.0;
    marker_array.markers[7].color.g = 0.6;
    marker_array.markers[7].color.b = 0.0;

    marker_array.markers[8].header.frame_id = "base";
    marker_array.markers[8].header.stamp = ros::Time::now();
    marker_array.markers[8].ns = "basic_shapes";
    marker_array.markers[8].id = 8;
    marker_array.markers[8].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[8].action = visualization_msgs::Marker::ADD;
    marker_array.markers[8].pose.position.x = ObjRobot[8].getTranslation().x();
    marker_array.markers[8].pose.position.y = ObjRobot[8].getTranslation().y();
    marker_array.markers[8].pose.position.z = ObjRobot[8].getTranslation().z();
    marker_array.markers[8].pose.orientation.x = ObjRobot[8].getQuatRotation().x();
    marker_array.markers[8].pose.orientation.y = ObjRobot[8].getQuatRotation().y();
    marker_array.markers[8].pose.orientation.z = ObjRobot[8].getQuatRotation().z();
    marker_array.markers[8].pose.orientation.w = ObjRobot[8].getQuatRotation().w();
    marker_array.markers[8].scale.x = 0.045;
    marker_array.markers[8].scale.y = 0.045;
    marker_array.markers[8].scale.z = 0.045;
    marker_array.markers[8].color.a = 1.0;
    marker_array.markers[8].color.r = 1.0;
    marker_array.markers[8].color.g = 0.0;
    marker_array.markers[8].color.b = 0.0;

    // marker_array.markers[7].header.frame_id = "base";
    // marker_array.markers[7].header.stamp = ros::Time::now();
    // marker_array.markers[7].ns = "basic_shapes";
    // marker_array.markers[7].id = 7;
    // marker_array.markers[7].type = visualization_msgs::Marker::SPHERE;
    // marker_array.markers[7].action = visualization_msgs::Marker::ADD;
    // marker_array.markers[7].pose.position.x = ObjRobot[7].getTranslation().x();
    // marker_array.markers[7].pose.position.y = ObjRobot[7].getTranslation().y();
    // marker_array.markers[7].pose.position.z = ObjRobot[7].getTranslation().z();
    // marker_array.markers[7].pose.orientation.x = 0.0;
    // marker_array.markers[7].pose.orientation.y = 0.0;
    // marker_array.markers[7].pose.orientation.z = 0.0;
    // marker_array.markers[7].pose.orientation.w = 1.0;
    // marker_array.markers[7].scale.x = 0.1;
    // marker_array.markers[7].scale.y = 0.1;
    // marker_array.markers[7].scale.z = 0.1; 
    // marker_array.markers[7].color.a = 0.7;
    // marker_array.markers[7].color.r = 0.7;
    // marker_array.markers[7].color.g = 0.7;
    // marker_array.markers[7].color.b = 0.0;

    marker_pub.publish(marker_array);
}

void LocalPlanner::PubJoints(void) {
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = "base";
    joint_state.name.resize(27);
    joint_state.position.resize(27);
    joint_state.name[0] = "head_joint1";
    joint_state.position[0] = 0;
    joint_state.name[1] = "head_joint2";
    joint_state.position[1] = 0;
    joint_state.name[2] = "left_joint1";
    joint_state.position[2] = 0;
    joint_state.name[3] = "left_joint2";
    joint_state.position[3] = 0;
    joint_state.name[4] = "left_joint3";
    joint_state.position[4] = 0;
    joint_state.name[5] = "left_joint4";
    joint_state.position[5] = 0;
    joint_state.name[6] = "left_joint5";
    joint_state.position[6] = 0;
    joint_state.name[7] = "left_joint6";
    joint_state.position[7] = 0;
    joint_state.name[8] = "left_jointf1";
    joint_state.position[8] = 0;
    joint_state.name[9] = "left_jointf2";
    joint_state.position[9] = 0;
    joint_state.name[10] = "left_jointf3";
    joint_state.position[10] = 0;
    joint_state.name[11] = "right_joint1";
    // joint_state.position[11] = -2.48131605;
    joint_state.position[11] = state(0, 0);
    // joint_state.velocity[11] = state(0, 1);
    joint_state.name[12] = "right_joint2";
    // joint_state.position[12] = -1.64973371;
    joint_state.position[12] = state(1, 0);
    // joint_state.velocity[12] = state(1, 1);
    joint_state.name[13] = "right_joint3";
    // joint_state.position[13] = -0.16230579;
    joint_state.position[13] = state(2, 0);
    // joint_state.velocity[13] = state(2, 1);
    joint_state.name[14] = "right_joint4";
    // joint_state.position[14] = 0.01278006;
    joint_state.position[14] = state(3, 0);
    // joint_state.velocity[14] = state(3, 1);
    joint_state.name[15] = "right_joint5";
    // joint_state.position[15] = 1.64867108;
    joint_state.position[15] = state(4, 0);
    // joint_state.velocity[15] = state(4, 1);
    joint_state.name[16] = "right_joint6";
    // joint_state.position[16] = 2.64416641;
    joint_state.position[16] = state(5, 0);
    // joint_state.velocity[16] = state(5, 1);
    if (Path.size() == 1) {
        joint_state.name[17] = "right_jointf1";
        joint_state.position[17] = 0.04;
        joint_state.name[18] = "right_jointf2";
        joint_state.position[18] = 0.04;
    } else {
        joint_state.name[17] = "right_jointf1";
        joint_state.position[17] = 0;
        joint_state.name[18] = "right_jointf2";
        joint_state.position[18] = 0;
    }
    // joint_state.name[17] = "right_jointf1";
    // joint_state.position[17] = 0;
    // joint_state.name[18] = "right_jointf2";
    // joint_state.position[18] = 0;
    joint_state.name[19] = "steering_joint1";
    joint_state.position[19] = 0;
    joint_state.name[20] = "wheel_joint1";
    joint_state.position[20] = 0;
    joint_state.name[21] = "steering_joint2";
    joint_state.position[21] = 0;
    joint_state.name[22] = "wheel_joint2";
    joint_state.position[22] = 0;
    joint_state.name[23] = "steering_joint3";
    joint_state.position[23] = 0;
    joint_state.name[24] = "wheel_joint3";
    joint_state.position[24] = 0;
    joint_state.name[25] = "steering_joint4";
    joint_state.position[25] = 0;
    joint_state.name[26] = "wheel_joint4";
    joint_state.position[26] = 0;

    joint_pub.publish(joint_state);

    // // client
    // airobots_calvin::ArmGoal g;
    // g.pos.clear();
    // g.pos.push_back(2.0);
    // g.pos.push_back(state(0, 0) * 180 / M_PI);
    // g.pos.push_back(state(1, 0) * 180 / M_PI);
    // g.pos.push_back(state(2, 0) * 180 / M_PI);
    // g.pos.push_back(state(3, 0) * 180 / M_PI);
    // g.pos.push_back(state(4, 0) * 180 / M_PI);
    // g.pos.push_back(state(5, 0) * 180 / M_PI);
    // std::cout << "goal: " << g.pos[0] << " " << g.pos[1] << " " << g.pos[2] << " " << g.pos[3] << " " << g.pos[4] << " " << g.pos[5] << " " << g.pos[6] << std::endl;
    // client_armR.pushGoal(g);

    bool gripper = false;
    if (Path.size() == 1)
        gripper = true;
    
    std::ofstream file;
    file.open("/home/aiRobots/Calvin/src/calvin/PathPlanning/src/joint_angles.txt", std::ios::app);

    if (file.is_open()) {
        file << state(0, 0) * 180 / M_PI << ", " << state(1, 0) * 180 / M_PI << ", " << state(2, 0) * 180 / M_PI << ", " << state(3, 0) * 180 / M_PI << ", " << state(4, 0) * 180 / M_PI << ", " << state(5, 0) * 180 / M_PI << ", " << gripper << ", " << std::endl;
    }
    file.close();
}

void LocalPlanner::UpdateDis(void) {
    obs_pos_list.clear();
    link_pos_list.clear();
    length_list.clear();

    // fcl::CollisionObjectf tree = fcl::CollisionObjectf(curr_tree);
    CollsionPair(0, ObjRobot[0], ObjRobot[4]);
    CollsionPair(1, ObjRobot[1], ObjRobot[4]);
    CollsionPair(2, ObjRobot[0], ObjRobot[2]);
    CollsionPair(3, ObjRobot[1], ObjRobot[2]);
    CollsionPair(4, ObjRobot[0], ObjRobot[3]);
    CollsionPair(5, ObjRobot[1], ObjRobot[3]);
    CollsionPair(6, ObjRobot[0], ObjRobot[6]);
    CollsionPair(7, ObjRobot[1], ObjRobot[6]);
    CollsionPair(8, ObjRobot[0], ObjRobot[7]);
    CollsionPair(9, ObjRobot[1], ObjRobot[7]);
}

void LocalPlanner::CollsionPair(const int& idx, fcl::CollisionObjectf& obj1, fcl::CollisionObjectf& obj2) {
    const int link = (idx % 2 == 0) ? 1 : 2;
    fcl::DistanceRequestf request;
    fcl::DistanceResultf result;
    request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    request.distance_tolerance = 1e-15;
    float d = fcl::distance(&obj1, &obj2, request, result);
    if (d < 0) {
        ROS_WARN("CollsionPair idx %d : failed", idx);
    }
    Eigen::Vector3f surface = Eigen::Vector3f(result.nearest_points[0][0], result.nearest_points[0][1], result.nearest_points[0][2]);
    float l = (key_joint_pos[link + 1] - key_joint_pos[link]).dot(surface - key_joint_pos[link]);
    length_list.push_back(l);
    // Eigen::Vector3f control_pos = key_joint_pos[link] + l * (key_joint_pos[link + 1] - key_joint_pos[link]) / (key_joint_pos[link + 1] - key_joint_pos[link]).norm();
    link_pos_list.push_back(surface);
    obs_pos_list.push_back(Eigen::Vector3f(result.nearest_points[1][0], result.nearest_points[1][1], result.nearest_points[1][2]));
}

float LocalPlanner::Move(const vector<float>& goal) {
    std::shared_ptr<RMPRoot> Root = getRMP(goal);
    Eigen::Matrix<float, 6, 2> state_dot = Dynamics(state, Root);
    Eigen::Matrix<float, 6, 2> new_state = state + state_dot * dt;
    const float max_q_dot = 0.2;
    for (int i = 0; i < 6; i++) {
        if (new_state(i, 1) > max_q_dot) {
            new_state(i, 1) = max_q_dot;
        } else if (new_state(i, 1) < -max_q_dot) {
            new_state(i, 1) = -max_q_dot;
        }
    }
    state = new_state;
    float error = 0.0;
    for (auto& Task : Root->children) {
        if (Task->name == "GJ") {
            error += Task->get_error();
        }
        for (auto& N : Task->children) {
            if (N->isLeaf) {
                error += N->get_error();
            } else {
                // for (auto& C : N->children) {
                //     error += C->get_error();
                // }
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
    Eigen::MatrixXf result = root->solve(x, x_dot);
    Eigen::Matrix<float, 6, 2> state_dot;
    state_dot << x_dot, result;
    return state_dot;
}

std::shared_ptr<RMPRoot> LocalPlanner::getRMP(const vector<float>& goal) {
    YAML::Node config = YAML::LoadFile("/home/aiRobots/Calvin/src/calvin/PathPlanning/src/RMP_para.yaml");
    Eigen::MatrixXf Pgoal(3, 1);
    Eigen::MatrixXf Qgoal(4, 1);
    Eigen::MatrixXf Jgoal(goal.size(), 1);
    if (goal.size() == 7) {
        Pgoal << goal[0], goal[1], goal[2];
        Qgoal << goal[3], goal[4], goal[5], goal[6];
    } else {
        Jgoal << goal[0], goal[1], goal[2], goal[3], goal[4], goal[5];
    }

    std::shared_ptr<RMPRoot> Root = std::make_shared<RMPRoot>("Root");
    std::shared_ptr<JointLimit> JL = std::make_shared<JointLimit>("JL", Root);
    Root->add_child(JL);
    std::shared_ptr<Damper> D = std::make_shared<Damper>("D", Root);
    Root->add_child(D);

    // Goal Reaching for joint
    float GJw_u = config["GJ"]["w_u"].as<float>();
    float GJw_l = config["GJ"]["w_l"].as<float>();
    std::shared_ptr<GoalAttractorUniT> GJ = std::make_shared<GoalAttractorUniT>("GJ", Root, Jgoal, GJw_u, GJw_l);
    Root->add_child(GJ);
    
    // // Goal Reaching for ee
    // std::shared_ptr<TransitionTaskSpace> eeT = std::make_shared<TransitionTaskSpace>("eeT", Root, 2, 0.32);
    // Root->add_child(eeT);
    // float GTw_u = config["GT"]["w_u"].as<float>();
    // float GTw_l = config["GT"]["w_l"].as<float>();
    // std::shared_ptr<GoalAttractorUniT> GT = std::make_shared<GoalAttractorUniT>("GT", eeT, Pgoal, GTw_u, GTw_l);
    // eeT->add_child(GT);

    // std::shared_ptr<RotationTaskSpace> eeR = std::make_shared<RotationTaskSpace>("eeR", Root, 0, 2, 0.32);
    // Root->add_child(eeR);
    // float GRw_u = config["GR"]["w_u"].as<float>();
    // float GRw_l = config["GR"]["w_l"].as<float>();
    // std::shared_ptr<GoalAttractorUniR> GR = std::make_shared<GoalAttractorUniR>("GR", eeR, 0, Qgoal, GRw_u, GRw_l);
    // eeR->add_child(GR);
    
    // std::shared_ptr<RotationTaskSpace> eeR1 = std::make_shared<RotationTaskSpace>("eeR1", Root, 1, 2, 0.32);
    // Root->add_child(eeR1);
    // std::shared_ptr<GoalAttractorUniR> GR1 = std::make_shared<GoalAttractorUniR>("GR1", eeR1, 1, Qgoal, Rw_u, Rw_l);
    // eeR1->add_child(GR1);
    // std::shared_ptr<RotationTaskSpace> eeR2 = std::make_shared<RotationTaskSpace>("eeR2", Root, 2, 2, 0.32);
    // Root->add_child(eeR2);
    // std::shared_ptr<GoalAttractorUniR> GR2 = std::make_shared<GoalAttractorUniR>("GR2", eeR2, 2, Qgoal, Rw_u, Rw_l);
    // eeR2->add_child(GR2);
    // std::shared_ptr<RotationTaskSpace> eeR3 = std::make_shared<RotationTaskSpace>("eeR3", Root, 3, 2, 0.32);
    // Root->add_child(eeR3);
    // std::shared_ptr<GoalAttractorUniR> GR3 = std::make_shared<GoalAttractorUniR>("GR3", eeR3, 3, Qgoal, Rw_u, Rw_l);
    // eeR3->add_child(GR3);

    // link1
    std::shared_ptr<TransitionTaskSpace> link1C = std::make_shared<TransitionTaskSpace>("link1C", Root, 1, length_list[0]);
    Root->add_child(link1C);
    std::shared_ptr<TransitionArbitraryPoint> link1CC = std::make_shared<TransitionArbitraryPoint>("link1CC", link1C, link_pos_list[0]);
    link1C->add_child(link1CC);
    float C1R = config["C1"]["R"].as<float>();
    float C1epslion = config["C1"]["epslion"].as<float>();
    std::shared_ptr<CollisionAvoidance> C1 = std::make_shared<CollisionAvoidance>("C1", link1CC, obs_pos_list[0], C1R, C1epslion);
    link1CC->add_child(C1);

    std::shared_ptr<TransitionTaskSpace> link1B = std::make_shared<TransitionTaskSpace>("link1B", Root, 1, length_list[2]);
    Root->add_child(link1B);
    std::shared_ptr<TransitionArbitraryPoint> link1CB = std::make_shared<TransitionArbitraryPoint>("link1CB", link1B, link_pos_list[2]);
    link1B->add_child(link1CB);
    float B1R = config["B1"]["R"].as<float>();
    float B1epslion = config["B1"]["epslion"].as<float>();
    std::shared_ptr<CollisionAvoidance> B1 = std::make_shared<CollisionAvoidance>("B1", link1CB, obs_pos_list[2], B1R, B1epslion);
    link1CB->add_child(B1);

    std::shared_ptr<TransitionTaskSpace> link1M = std::make_shared<TransitionTaskSpace>("link1M", Root, 1, length_list[4]);
    Root->add_child(link1M);
    std::shared_ptr<TransitionArbitraryPoint> link1CM = std::make_shared<TransitionArbitraryPoint>("link1CM", link1M, link_pos_list[4]);
    link1M->add_child(link1CM);
    float M1R = config["M1"]["R"].as<float>();
    float M1epslion = config["M1"]["epslion"].as<float>();
    std::shared_ptr<CollisionAvoidance> M1 = std::make_shared<CollisionAvoidance>("M1", link1CM, obs_pos_list[4], M1R, M1epslion);
    link1CM->add_child(M1);

    std::shared_ptr<TransitionTaskSpace> link1T = std::make_shared<TransitionTaskSpace>("link1T", Root, 1, length_list[6]);
    Root->add_child(link1T);
    std::shared_ptr<TransitionArbitraryPoint> link1CT = std::make_shared<TransitionArbitraryPoint>("link1CT", link1T, link_pos_list[6]);
    link1T->add_child(link1CT);
    float T1R = config["T1"]["R"].as<float>();
    float T1epslion = config["T1"]["epslion"].as<float>();
    std::shared_ptr<CollisionAvoidance> T1 = std::make_shared<CollisionAvoidance>("T1", link1CT, obs_pos_list[6], T1R, T1epslion);
    link1CT->add_child(T1);

    // std::shared_ptr<TransitionTaskSpace> link1b = std::make_shared<TransitionTaskSpace>("link1b", Root, 1, length_list[8]);
    // Root->add_child(link1b);
    // std::shared_ptr<TransitionArbitraryPoint> link1Cb = std::make_shared<TransitionArbitraryPoint>("link1Cb", link1b, link_pos_list[8]);
    // link1b->add_child(link1Cb);
    // float Box1R = config["Box1"]["R"].as<float>();
    // float Box1epslion = config["Box1"]["epslion"].as<float>();
    // std::shared_ptr<CollisionAvoidance> Box1 = std::make_shared<CollisionAvoidance>("Box1", link1Cb, obs_pos_list[8], Box1R, Box1epslion);
    // link1Cb->add_child(Box1);

    // std::shared_ptr<TransitionTaskSpace> link1c = std::make_shared<TransitionTaskSpace>("link1c", Root, 1, length_list[8]);
    // Root->add_child(link1c);
    // std::shared_ptr<TransitionArbitraryPoint> link1Cc = std::make_shared<TransitionArbitraryPoint>("link1Cc", link1c, link_pos_list[8]);
    // link1c->add_child(link1Cc);
    // float C21R = config["C21"]["R"].as<float>();
    // float C21epslion = config["C21"]["epslion"].as<float>();
    // std::shared_ptr<CollisionAvoidance> c1 = std::make_shared<CollisionAvoidance>("c1", link1Cc, obs_pos_list[8], C21R, C21epslion);
    // link1Cc->add_child(c1);


    // link2
    std::shared_ptr<TransitionTaskSpace> link2C = std::make_shared<TransitionTaskSpace>("link2C", Root, 2, length_list[1]);
    Root->add_child(link2C);
    std::shared_ptr<TransitionArbitraryPoint> link2CC = std::make_shared<TransitionArbitraryPoint>("link2CC", link2C, link_pos_list[1]);
    link2C->add_child(link2CC);
    float C2R = config["C2"]["R"].as<float>();
    float C2epslion = config["C2"]["epslion"].as<float>();
    std::shared_ptr<CollisionAvoidance> C2 = std::make_shared<CollisionAvoidance>("C2", link2CC, obs_pos_list[1], C2R, C2epslion);
    link2CC->add_child(C2);

    std::shared_ptr<TransitionTaskSpace> link2B = std::make_shared<TransitionTaskSpace>("link2B", Root, 2, length_list[3]);
    Root->add_child(link2B);
    std::shared_ptr<TransitionArbitraryPoint> link2CB = std::make_shared<TransitionArbitraryPoint>("link2CB", link2B, link_pos_list[3]);
    link2B->add_child(link2CB);
    float B2R = config["B2"]["R"].as<float>();
    float B2epslion = config["B2"]["epslion"].as<float>();
    std::shared_ptr<CollisionAvoidance> B2 = std::make_shared<CollisionAvoidance>("B2", link2CB, obs_pos_list[3], B2R, B2epslion);
    link2CB->add_child(B2);

    std::shared_ptr<TransitionTaskSpace> link2M = std::make_shared<TransitionTaskSpace>("link2M", Root, 2, length_list[5]);
    Root->add_child(link2M);
    std::shared_ptr<TransitionArbitraryPoint> link2CM = std::make_shared<TransitionArbitraryPoint>("link2CM", link2M, link_pos_list[5]);
    link2M->add_child(link2CM);
    float M2R = config["M2"]["R"].as<float>();
    float M2epslion = config["M2"]["epslion"].as<float>();
    std::shared_ptr<CollisionAvoidance> M2 = std::make_shared<CollisionAvoidance>("M2", link2CM, obs_pos_list[5], M2R, M2epslion);
    link2CM->add_child(M2);

    std::shared_ptr<TransitionTaskSpace> link2T = std::make_shared<TransitionTaskSpace>("link2T", Root, 2, length_list[7]);
    Root->add_child(link2T);
    std::shared_ptr<TransitionArbitraryPoint> link2CT = std::make_shared<TransitionArbitraryPoint>("link2CT", link2T, link_pos_list[7]);
    link2T->add_child(link2CT);
    float T2R = config["T2"]["R"].as<float>();
    float T2epslion = config["T2"]["epslion"].as<float>();
    std::shared_ptr<CollisionAvoidance> T2 = std::make_shared<CollisionAvoidance>("T2", link2CT, obs_pos_list[7], T2R, T2epslion);
    link2CT->add_child(T2);

    // std::shared_ptr<TransitionTaskSpace> link2b = std::make_shared<TransitionTaskSpace>("link2b", Root, 2, length_list[9]);
    // Root->add_child(link2b);
    // std::shared_ptr<TransitionArbitraryPoint> link2Cb = std::make_shared<TransitionArbitraryPoint>("link2Cb", link2b, link_pos_list[9]);
    // link2b->add_child(link2Cb);
    // float Box2R = config["Box2"]["R"].as<float>();
    // float Box2epslion = config["Box2"]["epslion"].as<float>();
    // std::shared_ptr<CollisionAvoidance> Box2 = std::make_shared<CollisionAvoidance>("Box2", link2Cb, obs_pos_list[9], Box2R, Box2epslion);
    // link2Cb->add_child(Box2);

    // std::shared_ptr<TransitionTaskSpace> link2c = std::make_shared<TransitionTaskSpace>("link2c", Root, 2, length_list[9]);
    // Root->add_child(link2c);
    // std::shared_ptr<TransitionArbitraryPoint> link2Cc = std::make_shared<TransitionArbitraryPoint>("link2Cc", link2c, link_pos_list[9]);
    // link2c->add_child(link2Cc);
    // float C22R = config["C22"]["R"].as<float>();
    // float C22epslion = config["C22"]["epslion"].as<float>();
    // std::shared_ptr<CollisionAvoidance> c2 = std::make_shared<CollisionAvoidance>("c2", link2Cc, obs_pos_list[9], C22R, C22epslion);
    // link2Cc->add_child(c2);
    
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
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker", 1);
    joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ball_sub = n.subscribe("/ball", 1, &OctreeGen::BallCallback, octreeGen);
    loop_rate = new ros::Rate(30);
    ros_thread = new std::thread(&LocalPlanner::Ros_spin, this);
    std::cout << "ROS initialized" << std::endl;
}
void LocalPlanner::Ros_spin(void) {
    while (!*terminated) {
        ros::spinOnce();
        loop_rate->sleep();
    }
    // ros::spin();
}
void LocalPlanner::StopROS(void) {
    *terminated = true;
    human_sub.shutdown();
    ball_sub.shutdown();
    ros_thread->join();
    delete this->ros_thread;
    delete loop_rate;
}