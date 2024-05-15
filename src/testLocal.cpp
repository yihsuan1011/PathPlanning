#include "LocalPlanner.h"

Eigen::Vector3f Q2Euler(const Eigen::Quaternionf& q)
{
    Eigen::Vector3f euler;
    euler[0] = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    euler[1] = 2 * atan2(sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z())), sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()))) - M_PI /2;
    euler[2] = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    return euler;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testGlobal");
    bool* terminated_flag = new bool(false);
    ArmRight* CRightArm = ArmRight::GetArmRight();
    CRightArm->SetTerminated_flag(terminated_flag);
    OctreeGen* COctreeGen = new OctreeGen();
    LocalPlanner* LP = LocalPlanner::GetLocalPlanner(CRightArm, COctreeGen);
    LP->Run();
    *terminated_flag = true;
    // CRightArm->SetTerminated_flag(terminated_flag);
    delete LP;
    delete COctreeGen;
    delete CRightArm;

    // float angles[3] = {0, 0, 0};
    // Eigen::Vector3f euler = Q2Euler(q) * Rad2Angle;
    // std::cout << "Euler from rotation matrix: " << euler[0] << ", " << euler[1] << ", " << euler[2] << std::endl;
    
    return 0;
}



