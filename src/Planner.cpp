#include "Planner.h"

Planner *Planner::inst_ = nullptr;

Planner *Planner::GetPlanner(Arm* carm) {
    if (inst_ == nullptr)
        inst_ = new Planner(Arm* carm);
    return inst_;
}

Planner::Planner(Arm* carm) {
    CArm = carm;
    LPlanner = LocalPlanner::GetLocalPlanner(Arm* carm);
}

Planner::~Planner() {
    delete LPlanner;
    inst_ = nullptr;
}

void Planner::ArmController(vector<float> goal) {
    float mode = goal[0];
    if (mode == 0.0) {
        if (CArm->GetTorqueEnableState()) {
            ROS_INFO("Turning off the torque");
            CArm->Stop();
        }
        else {
            ROS_INFO("Turning on the torque");
            CArm->Start();
        }
        // server.setSucceeded();
        return;
    }
    if (!CArm->GetTorqueEnableState()) {
        ROS_INFO("Torque is off, cannot move arm");
        // server.setAborted();
        return;
    }

    std::thread t1;
    if (mode == 1.0) {
        ROS_INFO("Executing in IK mode, arm reaching to [%f, %f, %f, %f, %f, %f]",
        goal->pos[1], goal->pos[2], goal->pos[3], goal->pos[4], goal->pos[5], goal->pos[6]);
        t1 = std::thread(/* Path Planning */, vector<float>(goal->pos.begin() + 1, goal->pos.end()));
    }
    else {
        ROS_INFO("Executing in motor mode, setting arm motor angles to [%f, %f, %f, %f, %f, %f]",
        goal->pos[1], goal->pos[2], goal->pos[3], goal->pos[4], goal->pos[5], goal->pos[6]);
        CArm->PSetArmVelocity(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
        t1 = std::thread(std::bind(&Arm::SetArmAngle, CArm), vector<float>(goal.begin() + 1, goal.end()));
    }
    while (!CArm->GetWorkingState());
    // while (CArm->GetWorkingState() || !CArm->CheckArmArrival()) {
    //     //Check for preemption
    //     // ROS_INFO("bruh");
    //     if (server.isPreemptRequested() || !ros::ok()) {
    //         ROS_INFO("%s: Preempted", action_name.c_str());
    //         CArm->ArmHalt();
    //         //set the action state to preempted
    //         server.setPreempted();
    //         success = false;
    //         break;
    //     }
    // }
    ROS_INFO("bruh_fin");
    t1.join();
}