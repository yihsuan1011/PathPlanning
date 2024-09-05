#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <functional>

class RMPNode : std::enable_shared_from_this<RMPNode>
{
public:
    RMPNode(std::string name, std::shared_ptr<RMPNode> parent,
            bool verbose = false);
    ~RMPNode();
    void add_child(std::shared_ptr<RMPNode> child);
    virtual void pushforward();
    virtual void pullback();
    virtual float get_error() {return 0.0;}
    // Eigen::MatrixXf resolve() = 0;
    // Node properties
    std::string name;
    std::weak_ptr<RMPNode> parent;
    std::vector<std::shared_ptr<RMPNode>> children;
    bool isLeaf;

    // Pushforward operator from parent to this node
    std::function<Eigen::MatrixXf(const Eigen::MatrixXf&)> psi;
    std::function<Eigen::MatrixXf(const Eigen::MatrixXf&)> J;
    std::function<Eigen::MatrixXf(const Eigen::MatrixXf&, const Eigen::MatrixXf&)> J_dot;
    // Eigen::MatrixXf (*psi)(const Eigen::MatrixXf&);
    // Eigen::MatrixXf(*J)(const Eigen::MatrixXf&);
    // Eigen::MatrixXf (*J_dot)(const Eigen::MatrixXf&, const Eigen::MatrixXf&);

    // Node state
    Eigen::MatrixXf x;
    Eigen::MatrixXf x_dot;

    // RMP
    Eigen::MatrixXf f;
    Eigen::MatrixXf a;
    Eigen::MatrixXf M;

    // Print the name of the node when applying operations if true
    bool verbose;

};

class RMPRoot : public RMPNode
{
public:
    RMPRoot(const std::string& name): RMPNode(name, nullptr) {children.reserve(8); isLeaf = false;}
    void set_root_state(const Eigen::MatrixXf& x, const Eigen::MatrixXf& x_dot);
    void pushforward(void) override;
    void pullback(void) override {RMPNode::pullback();}
    Eigen::MatrixXf resolve();
    Eigen::MatrixXf solve(const Eigen::MatrixXf& x, const Eigen::MatrixXf& x_dot);
};

class RMPLeaf : public RMPNode
{
public:
    RMPLeaf(const std::string& name, std::shared_ptr<RMPNode> parent)
    : RMPNode(name, parent) {isLeaf = true;}

    void eval_leaf();
    virtual void RMP_function() = 0;
    void pushforward() override {RMPNode::pushforward();}
    void pullback() override;
    void add_child(std::shared_ptr<RMPNode> child);
    void update_params();
    void update();
    float get_error() override;
    float err;

private:
};