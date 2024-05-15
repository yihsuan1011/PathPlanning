#pragma once

#include "RMP.h"
#include "Kinematics.h"

class CollisionAvoidance : public RMPLeaf
{
public:
    CollisionAvoidance(std::string name, std::shared_ptr<RMPNode> parent,
                       const Eigen::MatrixXf& c, float R = 1.0,
                       float epsilon = 0.2, float alpha = 1e-5,float eta = 0);
    ~CollisionAvoidance() {}
    
    void RMP_function(void) override;
private:
    float R, alpha, eta, epsilon;
    Eigen::MatrixXf c;
};

class GoalAttractorUniT : public RMPLeaf
{
public:
    GoalAttractorUniT(std::string name, std::shared_ptr<RMPNode> parent,
                      const Eigen::MatrixXf& y_g, float w_u = 10, float w_l = 0.1, 
                      float sigma = 1, float alpha = 1, float eta = 2, float gain = 1, float tol = 0.005);
    ~GoalAttractorUniT() {}
    
    void RMP_function(void) override;
private:
    float w_u, w_l, sigma, alpha, eta, gain, tol;
    Eigen::MatrixXf y_g;
};


class GoalAttractorUniR : public RMPLeaf
{
public:
    GoalAttractorUniR(std::string name, std::shared_ptr<RMPNode> parent,
                      const Eigen::MatrixXf& y_g = Eigen::MatrixXf::Zero(2, 1), float w_u = 10, float w_l = 0.1, 
                      float sigma = 1, float alpha = 1, float eta = 2, float gain = 1, float tol = 0.005);
    ~GoalAttractorUniR() {}
    
    void RMP_function(void) override;
private:
    float w_u, w_l, sigma, alpha, eta, gain, tol;
    Eigen::MatrixXf y_g;
};

class TransitionArbitraryPoint : public RMPNode
{
public:
    TransitionArbitraryPoint(std::string name, std::shared_ptr<RMPNode> parent, const Eigen::MatrixXf& point);
    ~TransitionArbitraryPoint() {}
    void pullback(void) override {RMPNode::pullback();}
private:
    Eigen::MatrixXf point;
};

class TransitionTaskSpace : public RMPNode
{
public:
    TransitionTaskSpace(std::string name, std::shared_ptr<RMPNode> parent, const int& link, const float& length);
    ~TransitionTaskSpace() {}
    void pullback(void) override {RMPNode::pullback();}
private:
    int link;
    float length;
    std::shared_ptr<KinematicsR> k;
};

class RotationTaskSpace : public RMPNode
{
public:
    RotationTaskSpace(std::string name, std::shared_ptr<RMPNode> parent, const int& link, const float& length);
    ~RotationTaskSpace() {}
    void pullback(void) override;
private:
    int link;
    float length;
    std::shared_ptr<KinematicsR> k;
};