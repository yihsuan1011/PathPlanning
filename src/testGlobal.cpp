#include "GlobalPlanner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testGlobal");
    bool* terminated_flag = new bool(false);
    ArmRight* CRightArm = ArmRight::GetArmRight();
    CRightArm->SetTerminated_flag(terminated_flag);
    GlobalPlanner* GP = GlobalPlanner::GetGlobalPlanner(CRightArm);
    GP->SetStart({300, -300, -300, 0, 90, 0});
    GP->SetGoal({300, -400, -200, 0, 90, 0});
    GP->Plan();
    *terminated_flag = true;
    // CRightArm->SetTerminated_flag(terminated_flag);
    
    return 0;
}





// #include <iostream>
// #include <ompl/base/SpaceInformation.h>
// #include <ompl/base/spaces/SE3StateSpace.h>
// #include <ompl/geometric/SimpleSetup.h>
// #include <ompl/geometric/planners/rrt/RRTConnect.h>

// class simplePlanner {
//     private:
//         ompl::base::StateSpacePtr space;
//         ompl::base::SpaceInformationPtr si;
//         ompl::base::ProblemDefinitionPtr pdef;
//     public:
//         simplePlanner() {
//             space = ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace());
//             ompl::base::RealVectorBounds bounds(3);
//             bounds.setLow(0, -1.0);
//             bounds.setLow(1, -1.0);
//             bounds.setLow(2, -1.5);
//             bounds.setHigh(0, 3.0);
//             bounds.setHigh(1, 1.0);
//             bounds.setHigh(2, 0.5);
//             space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
//             si = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(space));
//             si->setStateValidityChecker(std::bind(&simplePlanner::isStateValid, this, std::placeholders::_1));
//             si->setup();

//             pdef = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(si));
//             ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
//             obj->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
//             pdef->setOptimizationObjective(obj);
//         };
//         ~simplePlanner() {};
//         bool isStateValid(const ompl::base::State* state) {return true;};
//         void SetStart(vector<float> start) {
//             Eigen::Matrix3f R = Eigen::AngleAxisf(start[5] * Angle2Rad, Eigen::Vector3f::UnitZ())
//                                 * Eigen::AngleAxisf(start[4] * Angle2Rad, Eigen::Vector3f::UnitY())
//                                 * Eigen::AngleAxisf(start[3] * Angle2Rad, Eigen::Vector3f::UnitX()).toRotationMatrix();
//             Eigen::AngleAxisf aa;
//             aa.fromRotationMatrix(R);
//             ROS_INFO("aa: %f %f %f %f", aa.axis().x(), aa.axis().y(), aa.axis().z(), aa.angle());
//             ompl::base::ScopedState<ompl::base::SE3StateSpace> start_state(this->space);
//             start_state->setXYZ(start[0] / 1000, start[1] / 1000, start[2] / 1000);
//             start_state->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(aa.axis().x(), aa.axis().y(), aa.axis().z(), aa.angle());
//             pdef->addStartState(start_state);
//         };
//         void SetGoal(vector<float> goal) {
//             Eigen::Matrix3f R = Eigen::AngleAxisf(goal[5] * Angle2Rad, Eigen::Vector3f::UnitZ())
//                                 * Eigen::AngleAxisf(goal[4] * Angle2Rad, Eigen::Vector3f::UnitY())
//                                 * Eigen::AngleAxisf(goal[3] * Angle2Rad, Eigen::Vector3f::UnitX()).toRotationMatrix();
//             Eigen::AngleAxisf aa;
//             aa.fromRotationMatrix(R);
//             ROS_INFO("aa: %f %f %f %f", aa.axis().x(), aa.axis().y(), aa.axis().z(), aa.angle());
//             ompl::base::ScopedState<ompl::base::SE3StateSpace> goal_state(this->space);
//             goal_state->setXYZ(goal[0] / 1000, goal[1] / 1000, goal[2] / 1000);
//             goal_state->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(aa.axis().x(), aa.axis().y(), aa.axis().z(), aa.angle());
//             pdef->clearGoal();
//             pdef->setGoalState(goal_state);
//         };
//         void SetStart(float x, float y, float z) {
//             ompl::base::ScopedState<ompl::base::SE3StateSpace> start_state(this->space);
//             start_state->setXYZ(x / 1000, y / 1000, z / 1000);
//             pdef->clearStartStates();
//             pdef->addStartState(start_state);
//         };
//         void SetGoal(float x, float y, float z) {
//             ompl::base::ScopedState<ompl::base::SE3StateSpace> goal_state(this->space);
//             goal_state->setXYZ(x / 1000, y / 1000, z / 1000);
//             goal_state->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
//             pdef->clearGoal();
//             pdef->setGoalState(goal_state);
//         };
//         void Plan(void) {
//             ompl::geometric::InformedRRTstar* pType = new ompl::geometric::InformedRRTstar(si);
//             pType->setRange(0.05);
//             ompl::base::PlannerPtr planner(pType);
//             planner->setProblemDefinition(pdef);
//             planner->setup();
            
//             ompl::base::PlannerStatus solved = planner->solve(1);

//             if (solved) {
//                 std::cout << "Found solution:" << std::endl;
//                 ompl::geometric::PathGeometric* path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
//                 path->printAsMatrix(std::cout);
//             } else {
//                 std::cout << "No solution found" << std::endl;
//             }
//         };
// };

// int main() {
//     simplePlanner* sp = new simplePlanner();
//     sp->SetStart({300, -300, -300, 0, 90, 0});
//     sp->SetGoal({300, -400, -200, 0, 90, 0});
//     // sp->SetStartAndGoal({300, -300, -300, 0, 90, 0}, {300, -400, -200, 0, 90, 0});
//     // sp->SetStart(300, -300, -300);
//     // sp->SetGoal(300, -400, -200);
//     sp->Plan();
//     return 0;
// }





// bool isStateValid(const ompl::base::State* state) {
//     return true;
// }

// int main() {
//     ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());
 
//     ompl::base::RealVectorBounds bounds(3);
//     bounds.setLow(0, -1.0);
//     bounds.setLow(1, -1.0);
//     bounds.setLow(2, -1.5);
//     bounds.setHigh(0, 3.0);
//     bounds.setHigh(1, 1.0);
//     bounds.setHigh(2, 0.5);
//     space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

//     ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
//     si->setStateValidityChecker(isStateValid);
//     si->setup();

//     ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
//     ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
//     obj->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
//     pdef->setOptimizationObjective(obj);

//     ompl::base::ScopedState<ompl::base::SE3StateSpace> start(space), goal(space);
//     start->setXYZ(0, 0, 0);
//     start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
//     goal->setXYZ(1, 1, 1);
//     goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
//     pdef->setStartAndGoalStates(start, goal);

//     // set the planner
//     ompl::geometric::InformedRRTstar* pType = new ompl::geometric::InformedRRTstar(si);
//     pType->setRange(0.05);
//     ompl::base::PlannerPtr planner(pType);
//     planner->setProblemDefinition(pdef);
//     planner->setup();
    
//     ompl::base::PlannerStatus solved = planner->solve(1);

//     if (solved) {
//         std::cout << "Found solution:" << std::endl;
//         ompl::geometric::PathGeometric* path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
//         path->printAsMatrix(std::cout);
//     } else {
//         std::cout << "No solution found" << std::endl;
//     }

//     return 0;
// }
