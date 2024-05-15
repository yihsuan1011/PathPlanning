#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <deque>
#include <forward_list>
#include <array>
#include <eigen3/Eigen/Core>
#include <thread>
#include <ros/ros.h>
// #include <actionlib/server/simple_action_server.h>
#include "Robot/Robot.h"

class Planner
{
private:
    Planner(Arm* carm);
    static Planner *inst_;
    Arm* CArm;
    // LocalPlanner* LPlanner;
    // GlobalPlanner* GPlanner;

public:
    static Planner *GetPlanner(Arm* carm);
    ~Planner();
    forward_list<array<float, 3>> path;
    void ArmController(vector<float> goal);
};