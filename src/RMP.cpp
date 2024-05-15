#include "RMP.h"

// RMPNode
RMPNode::RMPNode(std::string name, std::shared_ptr<RMPNode> parent,
                 bool verbose)
: name(name), parent(parent), verbose(verbose), isLeaf(false) {
    // std::cout << "Creating node " << name << std::endl;
    // if (parent) {
    //     parent->add_child(shared_from_this());
    // }
}

RMPNode::~RMPNode() {
    for (auto& child : children) {
        child->parent = nullptr;
    }
}

void RMPNode::add_child(std::shared_ptr<RMPNode> child) {

    children.push_back(child);
}

void RMPNode::pushforward() {
    if (verbose) {
        std::cout << name << ": pushforward" << std::endl;
    }
    if (parent) {
        std::cout << name << ": pushforward" << std::endl;
        x = psi(parent->x);
        Eigen::MatrixXf J_matrix = J(parent->x);
        x_dot = J_matrix * parent->x_dot.block(0, 0, J_matrix.cols(), 1);
    }
    for (auto& child : children) {
        child->pushforward();
    }
}

void RMPNode::pullback() {
    std::cout << name << ": pullback" << std::endl;
    for (auto& child : children) {
        child->pullback();
    }
    if (verbose) {
        std::cout << name << ": pullback" << std::endl;
    }
    Eigen::MatrixXf f_total = Eigen::MatrixXf::Zero(x.rows(), x.cols());
    Eigen::MatrixXf M_total = Eigen::MatrixXf::Zero(std::max(x.rows(), x.rows()), std::max(x.rows(), x.rows()));

    for (auto& child : children) {
        Eigen::MatrixXf J_child = child->J(x);
        Eigen::MatrixXf J_dot_child = child->J_dot(x, x_dot);
        if (child->f.size() > 0 && child->M.size() > 0) {
            Eigen::MatrixXf f_child = J_child.transpose() * child->f;
            if (f_child.rows() != x.rows()) {
                Eigen::MatrixXf temp = Eigen::MatrixXf::Zero(x.rows(), f_child.cols());
                temp.topRows(f_child.rows()) = f_child;
                f_child = temp;
            }
            f_total += f_child;
            Eigen::MatrixXf M_child = J_child.transpose() * child->M * J_child;
            M_total += M_child;
            // M_total += J_child.transpose() * child->M * J_child;
        }
    }
    f = f_total;
    M = M_total;
    std::cout << name << ": pullback done" << std::endl;
}

// RMPRoot
void RMPRoot::set_root_state(const Eigen::MatrixXf& x, const Eigen::MatrixXf& x_dot) {
    this->x = x;
    this->x_dot = x_dot;
}

void RMPRoot::pushforward() {
    if (verbose) {
        std::cout << name << ": pushforward" << std::endl;
    }
    for (auto& child : children) {
        child->pushforward();
    }
}

Eigen::MatrixXf RMPRoot::resolve() {
    if (verbose) {
        std::cout << name << ": resolve" << std::endl;
    }
    Eigen::MatrixXf a = (M.completeOrthogonalDecomposition().pseudoInverse() * f);    // checke the calculation
    return a;
}

Eigen::MatrixXf RMPRoot::solve(const Eigen::MatrixXf& x, const Eigen::MatrixXf& x_dot) {
    std::cout << "Solving RMP" << std::endl;
    set_root_state(x, x_dot);
    std::cout << "done setting root state" << std::endl;
    pushforward();
    std::cout << "done pushforward" << std::endl;
    pullback();
    std::cout << "done pullback" << std::endl;
    return resolve();
}

// RMPLeaf
void RMPLeaf::eval_leaf() {
    std::cout << name << ": eval_leaf" << std::endl;
    RMP_function();
}

void RMPLeaf::pullback() {
    std::cout << name << ": pullback" << std::endl;
    if (verbose) {
        std::cout << name << ": pullback" << std::endl;
    }
    eval_leaf();
    std::cout << name << ": pullback eval done" << std::endl; 
}

void RMPLeaf::add_child(std::shared_ptr<RMPNode> child) {
    std::cout << "Cannot add child to leaf node" << std::endl;
}

void RMPLeaf::update_params() {}

void RMPLeaf::update() {
    update_params();
    pushforward();    
}

float RMPLeaf::get_error() {
    return err;
}