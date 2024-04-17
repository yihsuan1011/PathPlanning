#include "LocalPlanner.h"

LocalPlanner* LocalPlanner::inst_ = nullptr;

LocalPlanner* LocalPlanner::GetLocalPlanner(Arm* carm) {
    if (inst_ == nullptr)
        inst_ = new LocalPlanner(Arm* carm);
    return inst_;
}

LocalPlanner::LocalPlanner(Arm* carm) {
    CArm = carm;
    curr = Eigen::Vector3f(CArm->GetCurrentPosition(0), CArm->GetCurrentPosition(1), CArm->GetCurrentPosition(2));
    last = curr;
    // goal = g;
    dt = 1 / 30;
    human_IDlist = {126, 13, 122, 118, 112, 113, 114, 115, 15, 16, 17, 18};
    InitialROS();
}

LocalPlanner::~LocalPlanner() {
    StopROS();
    inst_ = nullptr;
}

void LocalPlanner::UpdateState(void) {
    // t1 = ros::Time::now();
    // if (t2 != 0)
    //     dt = t1 - t2;
    // t2 = t1;
    std::shared_ptr<fcl::CollisionGeometryf> end_shape (new fcl::Spheref(25));
    fcl::CollisionObjectf end = fcl::CollisionObjectf(end_shape);
    end.setTranslation(fcl::Vector3f(curr[0], curr[1], curr[2]));
    CreateFCL();
    fcl::DistanceRequestf request;
    fcl::DistanceResultf result;
    last_obs.clear();
    last_obs = curr_obs;
    curr_obs.clear();
    for (auto& ob : human_body) {
        fcl::distance(&end, &ob, request, result);
        curr_obs.push_back(Eigen::Vector3f(result.nearest_points[1][0], result.nearest_points[1][1], result.nearest_points[1][2]));
    }
    APF_acc();
    curr = curr + ((curr - last) / dt + acc * dt);
    // TODO: set curr
    CArm->TrajectoryPlanning(0, 90, 0, curr[0], curr[1], curr[2]);
    last = curr;
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
    vector<float> FCLsize = {125, (human[4]-human[8]).norm()/2, ((human[4]+human[8])/2-(human[2]+human[3])/2).norm(), 50, 100,
                             (human[4]-human[5]).norm(), (human[5]-human[6]).norm(), 
                             (human[8]-human[9]).norm(), (human[9]-human[10]).norm()};
    vector<std::shared_ptr<fcl::CollisionGeometryf>> human_shape;
    // human_shape[0] = std::make_shared<fcl::CollisionGeometryf>(fcl::Spheref(FCLsize[0]));
    human_shape[0] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Spheref(FCLsize[0]));
    human_shape[1] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Cylinderf(FCLsize[1], FCLsize[2]));
    human_shape[2] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Cylinderf(FCLsize[3], FCLsize[5]));
    human_shape[3] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Cylinderf(FCLsize[3], FCLsize[6]));
    human_shape[4] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Spheref(FCLsize[4]));
    human_shape[5] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Cylinderf(FCLsize[3], FCLsize[7]));
    human_shape[6] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Cylinderf(FCLsize[3], FCLsize[8]));
    human_shape[7] = std::shared_ptr<fcl::CollisionGeometryf> (new fcl::Spheref(FCLsize[4]));
    vector<Eigen::Vector3f> eigen_T = {2*human[0]-human[1], (human[4]+human[8])/2, human[4], human[5], human[7], human[8], human[9], human[11]};
    vector<Eigen::Vector3f> eigen_q = {((human[2]+human[3])/2-(human[4]+human[8])/2)/((human[2]+human[3])/2-(human[4]+human[8])/2).norm(), 
                                       (human[5]-human[4])/(human[5]-human[4]).norm(), (human[6]-human[5])/(human[6]-human[5]).norm(), 
                                       (human[9]-human[8])/(human[9]-human[8]).norm(), (human[10]-human[9])/(human[10]-human[9]).norm()};
    vector<fcl::Vector3f> FCL_T;
    for (auto& T : eigen_T) {
        FCL_T.push_back(fcl::Vector3f(T[0], T[1], T[2]));
    }
    vector<fcl::Quaternionf> FCL_q;
    for (auto& q : eigen_q) {
        fcl::Quaternionf tmp_q;
        FCL_q.push_back(fcl::Quaternionf(0, q[0], q[1], q[2]));
    }
    human_body.clear();
    for (size_t i = 0; i < human_shape.size(); i++) {
        human_body[i] = fcl::CollisionObjectf(human_shape[i]);
    }
    human_body[0].setTranslation(FCL_T[0]);
    human_body[1].setTransform(FCL_q[0], FCL_T[1]);
    human_body[2].setTransform(FCL_q[1], FCL_T[2]);
    human_body[3].setTransform(FCL_q[2], FCL_T[3]);
    human_body[4].setTranslation(FCL_T[4]);
    human_body[5].setTransform(FCL_q[3], FCL_T[5]);
    human_body[6].setTransform(FCL_q[4], FCL_T[6]);
    human_body[7].setTranslation(FCL_T[7]);
}
void LocalPlanner::HumanCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    human.clear();
    human.resize(12, Eigen::Vector3f::Zero());
    Eigen::Vector3f p;
    for (auto& marker : msg->markers) {
        for (int j = 0; j < 12; j++) {
            if (human_IDlist[j] == marker.id)
                p = {static_cast<float>(marker.pose.position.x), static_cast<float>(marker.pose.position.y), static_cast<float>(marker.pose.position.z)};
            human[j] = p / 1000;
        }
    }
    UpdateState();
}

void LocalPlanner::InitialROS(void) {
    human_sub = n.subscribe("/body_tracking_data", 1, &LocalPlanner::HumanCallback, this);
}
void LocalPlanner::Ros_spin(void) {
    ros::spin();
}
void LocalPlanner::StopROS(void) {
    human_sub.shutdown();
}