#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>

class KinematicsR
{
public:
    KinematicsR(int link, float length) : link(link), length(length){
        if (link == 1) {
            UPPER_LINK_LENGTH_ = length;
        } else if (link == 2) {
            END_EFFECTOR_LENGTH_ = length;
        }
    };
    ~KinematicsR() {};
    
    Eigen::MatrixXf getPosFK(const Eigen::MatrixXf& q) {
        Eigen::Matrix4f Tfk = getForwardKinematics(q);
        Eigen::MatrixXf P(3, 1);
        P << Tfk(0, 3), Tfk(1, 3), Tfk(2, 3);
        return P;
    }

    Eigen::MatrixXf getOriFK(const Eigen::MatrixXf& q) {
        Eigen::Matrix4f Tfk = getForwardKinematics(q);
        Eigen::Matrix3f Rfk = Tfk.block<3, 3>(0, 0);
        Eigen::MatrixXf Q(4, 1);
        // float n = 0.5 * sqrt(1 + Rfk(0, 0) + Rfk(1, 1) + Rfk(2, 2));
        // float e1 = 0.5 * std::copysign(1, Rfk(2, 1) - Rfk(1, 2)) * sqrt(1 + Rfk(0, 0) - Rfk(1, 1) - Rfk(2, 2));
        // float e2 = 0.5 * std::copysign(1, Rfk(0, 2) - Rfk(2, 0)) * sqrt(1 - Rfk(0, 0) + Rfk(1, 1) - Rfk(2, 2));
        // float e3 = 0.5 * std::copysign(1, Rfk(1, 0) - Rfk(0, 1)) * sqrt(1 - Rfk(0, 0) - Rfk(1, 1) + Rfk(2, 2));
        Eigen::Quaternionf EigenQ(Rfk);
        // float T = Rfk(0, 0) + Rfk(1, 1) + Rfk(2, 2);
        // float M = std::max(T, std::max(Rfk(0, 0), std::max(Rfk(1, 1), Rfk(2, 2)));
        // float qmax = sqrt(1 - T + 2 * M) / 2;
        // if (M == Rfk(0, 0)) {
        //     Q << sqrt(1 + 2 * Rfk(0, 0) - T) / 2, (Rfk(1, 0) + Rfk(0, 1)) / (4 * qmax), (Rfk(2, 0) + Rfk(0, 2)) / (4 * qmax), (Rfk(2, 1) - Rfk(1, 2)) / (4 * qmax);
        // } else if (M == Rfk(1, 1)) {
        //     Q << (Rfk(1, 0) + Rfk(0, 1)) / (4 * qmax), sqrt(1 + 2 * Rfk(1, 1) - T) / 2, (Rfk(2, 1) + Rfk(1, 2)) / (4 * qmax), (Rfk(0, 2) - Rfk(2, 0)) / (4 * qmax);
        // } else if (M == Rfk(2, 2)) {
        //     Q << (Rfk(2, 0) + Rfk(0, 2)) / (4 * qmax), (Rfk(2, 1) + Rfk(1, 2)) / (4 * qmax), sqrt(1 + 2 * Rfk(2, 2) - T) / 2, (Rfk(1, 0) - Rfk(0, 1)) / (4 * qmax);
        // } else {
        //     Q << (Rfk(2, 1) - Rfk(1, 2)) / (4 * qmax), (Rfk(0, 2) - Rfk(2, 0)) / (4 * qmax), (Rfk(1, 0) - Rfk(0, 1)) / (4 * qmax), sqrt(1 + T) / 2;
        // }
        if (EigenQ.w() < 0) EigenQ.coeffs() *= -1; // w is positive
        Q << EigenQ.w(), EigenQ.x(), EigenQ.y(), EigenQ.z();
        return Q;
    }

    Eigen::MatrixXf getPosJ(const Eigen::MatrixXf& q) {
        Eigen::MatrixXf J = getJacobian(q);
        Eigen::MatrixXf Jpos = J.block(0, 0, 3, J.cols());
        return Jpos;
    }

    Eigen::MatrixXf getOriJ(const Eigen::MatrixXf& q) {
        Eigen::MatrixXf J = getJacobian(q);
        Eigen::MatrixXf Jori = J.block(3, 0, 3, J.cols());
        return Jori;
    }

    Eigen::Matrix4f getForwardKinematics(const Eigen::MatrixXf& q, bool isAngle = false) {
        float trans = isAngle ? M_PI / 180 : 1;
        float J1 = q(0, 0) * trans;
        float J2 = q(1, 0) * trans;
        float J3 = q(2, 0) * trans;

        Eigen::Matrix4f Tb0;
        Tb0 << 1, 0, 0, 0,
            0, -1, 0, -CORE_LINK_LENGTH_,
            0, 0, -1, 0,
            0, 0, 0, 1;
        Eigen::Matrix4f T01 = GetTransformMatrix_Craig(0, 0, J1, 0);
        Eigen::Matrix4f T12 = GetTransformMatrix_Craig(-M_PI_2, 0, J2 - M_PI_2, 0);
        Eigen::Matrix4f T225 = GetTransformMatrix_Craig(-M_PI_2, 0, -M_PI_2, 0);
        Eigen::Matrix4f T253 = GetTransformMatrix_Craig(0, SHOULDER_LINK_LENGTH_, J3 + M_PI_2, 0);
        Eigen::Matrix4f T335 = GetTransformMatrix_Craig(0, UPPER_LINK_LENGTH_, M_PI_2, 0);

        Eigen::Matrix4f fkpos;
        fkpos = Tb0 * T01 * T12 * T225 * T253 * T335;
        if (link > 1) {
            float J4 = q(3, 0) * trans;
            float J5 = q(4, 0) * trans;
            float J6 = q(5, 0) * trans;
            Eigen::Matrix4f T354 = GetTransformMatrix_Craig(M_PI_2, 0, J4 + M_PI_2, 0);
            Eigen::Matrix4f T45 = GetTransformMatrix_Craig(-M_PI_2, 0, J5, 0);
            Eigen::Matrix4f T56 = GetTransformMatrix_Craig(M_PI_2, 0, J6, 0);
            Eigen::Matrix4f T6E;
            T6E << 0, 0, 1, 0,
                0, -1, 0, 0,
                1, 0, 0, END_EFFECTOR_LENGTH_,
                0, 0, 0, 1;

            fkpos = fkpos * T354 * T45 * T56 * T6E;
        }
        return fkpos;
    }

    Eigen::MatrixXf getJacobian(const Eigen::MatrixXf& q, bool isAngle = false) {
        float trans = isAngle ? M_PI / 180 : 1;
        /* get motor delta angle*/
        float J1 = q(0, 0) * trans;
        float J2 = q(1, 0) * trans;
        float J3 = q(2, 0) * trans;
        Eigen::Matrix4f Tb0;
        Tb0 << 1, 0, 0, 0,
            0, -1, 0, -CORE_LINK_LENGTH_,
            0, 0, -1, 0,
            0, 0, 0, 1;
        Eigen::Matrix4f T01 = GetTransformMatrix_Craig(0, 0, J1, 0);
        Eigen::Matrix4f T12 = GetTransformMatrix_Craig(-M_PI_2, 0, J2 - M_PI_2, 0);
        Eigen::Matrix4f T225 = GetTransformMatrix_Craig(-M_PI_2, 0, -M_PI_2, 0);
        Eigen::Matrix4f T253 = GetTransformMatrix_Craig(0, SHOULDER_LINK_LENGTH_, J3 + M_PI_2, 0);
        Eigen::Matrix4f T335 = GetTransformMatrix_Craig(0, UPPER_LINK_LENGTH_, M_PI_2, 0);
        Eigen::Matrix4f Tb1 = Tb0 * T01;
        Eigen::Matrix4f Tb2 = Tb1 * T12;
        Eigen::Matrix4f Tb3 = Tb2 * T225 * T253;
        Eigen::Matrix4f Tb35 = Tb3 * T335;
        Eigen::Vector3f Pb0;
        Eigen::Vector3f Pb1;
        Eigen::Vector3f Pb2;
        Eigen::Vector3f Pb3;
        Eigen::Vector3f Pb35;
        Pb0 << 0, 0, 0;
        Pb1 << Tb1(0, 3), Tb1(1, 3), Tb1(2, 3);
        Pb2 << Tb2(0, 3), Tb2(1, 3), Tb2(2, 3);
        Pb3 << Tb3(0, 3), Tb3(1, 3), Tb3(2, 3);
        Pb35 << Tb35(0, 3), Tb35(1, 3), Tb35(2, 3);
        Eigen::Vector3f Zb0;
        Eigen::Vector3f Zb1;
        Eigen::Vector3f Zb2;
        Eigen::Vector3f Zb3;
        Zb0 << 0, 0, 1;
        Zb1 << Tb1(0, 2), Tb1(1, 2), Tb1(2, 2);
        Zb2 << Tb2(0, 2), Tb2(1, 2), Tb2(2, 2);
        Zb3 << Tb3(0, 2), Tb3(1, 2), Tb3(2, 2);
        if (link > 1) {
            float J4 = q(3, 0) * trans;
            float J5 = q(4, 0) * trans;
            float J6 = q(5, 0) * trans;
            Eigen::Matrix4f T354 = GetTransformMatrix_Craig(M_PI_2, 0, J4 + M_PI_2, 0);
            Eigen::Matrix4f T45 = GetTransformMatrix_Craig(-M_PI_2, 0, J5, 0);
            Eigen::Matrix4f T56 = GetTransformMatrix_Craig(M_PI_2, 0, J6, 0);
            Eigen::Matrix4f T6E;
            T6E << 0, 0, 1, 0,
                0, -1, 0, 0,
                1, 0, 0, END_EFFECTOR_LENGTH_,
                0, 0, 0, 1;
            Eigen::Matrix4f Tb4 = Tb3 * T335 * T354;
            Eigen::Matrix4f Tb5 = Tb4 * T45;
            Eigen::Matrix4f Tb6 = Tb5 * T56;
            Eigen::Matrix4f TbE = Tb6 * T6E;
            Eigen::Vector3f Pb4;
            Eigen::Vector3f Pb5;
            Eigen::Vector3f Pb6;
            Eigen::Vector3f PbE;
            Pb4 << Tb4(0, 3), Tb4(1, 3), Tb4(2, 3);
            Pb5 << Tb5(0, 3), Tb5(1, 3), Tb5(2, 3);
            Pb6 << Tb6(0, 3), Tb6(1, 3), Tb6(2, 3);
            PbE << TbE(0, 3), TbE(1, 3), TbE(2, 3);
            Eigen::Vector3f Zb4;
            Eigen::Vector3f Zb5;
            Eigen::Vector3f Zb6;
            Eigen::Vector3f Zb7;
            Zb4 << Tb4(0, 2), Tb4(1, 2), Tb4(2, 2);
            Zb5 << Tb5(0, 2), Tb5(1, 2), Tb5(2, 2);
            Zb6 << Tb6(0, 2), Tb6(1, 2), Tb6(2, 2);

            Eigen::Vector3f Jvb1 = Zb1.cross(PbE - Pb1);
            Eigen::Vector3f Jvb2 = Zb2.cross(PbE - Pb2);
            Eigen::Vector3f Jvb3 = Zb3.cross(PbE - Pb3);
            Eigen::Vector3f Jvb4 = Zb4.cross(PbE - Pb4);
            Eigen::Vector3f Jvb5 = Zb5.cross(PbE - Pb5);
            Eigen::Vector3f Jvb6 = Zb6.cross(PbE - Pb6);
            Eigen::MatrixXf jacobian_matrix_(6, 6);
            jacobian_matrix_ << Jvb1(0, 0), Jvb2(0, 0), Jvb3(0, 0), Jvb4(0, 0), Jvb5(0, 0), Jvb6(0, 0), 
                                Jvb1(1, 0), Jvb2(1, 0), Jvb3(1, 0), Jvb4(1, 0), Jvb5(1, 0), Jvb6(1, 0), 
                                Jvb1(2, 0), Jvb2(2, 0), Jvb3(2, 0), Jvb4(2, 0), Jvb5(2, 0), Jvb6(2, 0), 
                                Zb1(0, 0), Zb2(0, 0), Zb3(0, 0), Zb4(0, 0), Zb5(0, 0), Zb6(0, 0), 
                                Zb1(1, 0), Zb2(1, 0), Zb3(1, 0), Zb4(1, 0), Zb5(1, 0), Zb6(1, 0), 
                                Zb1(2, 0), Zb2(2, 0), Zb3(2, 0), Zb4(2, 0), Zb5(2, 0), Zb6(2, 0);
            return jacobian_matrix_;
        } else {
            Eigen::Vector3f Jvb1 = Zb1.cross(Pb35 - Pb1);
            Eigen::Vector3f Jvb2 = Zb2.cross(Pb35 - Pb2);
            Eigen::Vector3f Jvb3 = Zb3.cross(Pb35 - Pb3);
            Eigen::MatrixXf jacobian_matrix_(6, 3);
            jacobian_matrix_ << Jvb1(0, 0), Jvb2(0, 0), Jvb3(0, 0), 
                                Jvb1(1, 0), Jvb2(1, 0), Jvb3(1, 0), 
                                Jvb1(2, 0), Jvb2(2, 0), Jvb3(2, 0),
                                Zb1(0, 0), Zb2(0, 0), Zb3(0, 0), 
                                Zb1(1, 0), Zb2(1, 0), Zb3(1, 0), 
                                Zb1(2, 0), Zb2(2, 0), Zb3(2, 0);
                                

            return jacobian_matrix_;
        }
    }

private:
    Eigen::MatrixXf GetTransformMatrix_Craig(const float &alpha_1, const float &a_1, const float &theta, const float &d) {
        Eigen::MatrixXf transform_matrix(4, 4);
        transform_matrix << cos(theta), -sin(theta), 0, a_1,
            sin(theta) * cos(alpha_1), cos(theta) * cos(alpha_1), -sin(alpha_1), -d * sin(alpha_1),
            sin(theta) * sin(alpha_1), cos(theta) * sin(alpha_1), cos(alpha_1), cos(alpha_1) * d,
            0, 0, 0, 1;

        return transform_matrix;
    }

    Eigen::Vector3f GetOrientationError(const Eigen::Matrix3f& Rd, const Eigen::Matrix3f& Re) {
        Eigen::Vector3f error;
        error = (Re.col(0).cross(Rd.col(0)) + Re.col(1).cross(Rd.col(1)) + Re.col(2).cross(Rd.col(2))) / 2;
        return error;
    }

    int link = 2;
    float length = 0.32;
    float CORE_LINK_LENGTH_ = 0.175;
	float SHOULDER_LINK_LENGTH_ = 0.122;
	float UPPER_LINK_LENGTH_ = 0.257;
	float END_EFFECTOR_LENGTH_ = 0.32;
};