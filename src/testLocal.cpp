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
    int mode = 0;
    // std::cout << "Choose mode: 0 for testLocal, 1 for Kinematics test : ";
    // std::cin >> mode;
    if (mode == 1) {
        KinematicsR* CKinematicsR = new KinematicsR(2, 0.32);
        Eigen::MatrixXf q(6, 1);
        q << -0.36919227, 1.65581407, -0.41166997, 3.10750463, 1.6486961, 3.09778644;
        // q << 0, 0, 0, 0, 0, 0;
        // std::cout << "Enter q: ";
        // for (int i = 0; i < 6; i++) {
        //     std::cin >> q(i, 0);
        // }
        Eigen::MatrixXf pos = CKinematicsR->getPosFK(q);
        Eigen::MatrixXf ori = CKinematicsR->getOriFK(q);
        Eigen::MatrixXf posJ = CKinematicsR->getPosJ(q);
        Eigen::MatrixXf oriJ = CKinematicsR->getOriJ(q);
        std::cout << "pos: " << std::endl << pos << std::endl;
        std::cout << "ori: " << std::endl << ori << std::endl;
        std::cout << "posJ: " << std::endl << posJ << std::endl;
        std::cout << "oriJ: " << std::endl << oriJ << std::endl;

        delete CKinematicsR;

    } else {
        ros::init(argc, argv, "testLocal");

        bool* terminated_flag = new bool(false);
        ArmRight* CRightArm = ArmRight::GetArmRight();
        CRightArm->SetTerminated_flag(terminated_flag);
        OctreeGen* COctreeGen = new OctreeGen();
        LocalPlanner* LP = LocalPlanner::GetLocalPlanner(terminated_flag, CRightArm, COctreeGen);
        LP->Run();
        // *terminated_flag = true;
        delete LP;
        delete COctreeGen;
        delete CRightArm;

    }

    return 0;
}



