# include "ActualNode.h"
# include <typeinfo>

CollisionAvoidance::CollisionAvoidance(std::string name, std::shared_ptr<RMPNode> parent,
                                       const Eigen::MatrixXf& c, float R,
                                       float epsilon, float alpha, float eta) 
: RMPLeaf(name, parent), c(c), R(R), alpha(alpha), eta(eta), epsilon(epsilon) {
    // c 3 * 1
    // y 3 * 1
    // y_dot 3 * 1

    psi = [this](const Eigen::MatrixXf& y) {
        float norm_y_c = (y - this->c).norm();
        return Eigen::MatrixXf::Constant(1, 1, norm_y_c / this->R - 1);
    }; // 1 * 1

    J = [this](const Eigen::MatrixXf& y) {
        float norm_y_c = (y - this->c).norm();
        return (1.0 / norm_y_c) * (y - this->c).transpose() / this->R;
    }; // 1 * 3

    J_dot = [this](const Eigen::MatrixXf& y, const Eigen::MatrixXf& y_dot) {
        Eigen::MatrixXf I = Eigen::MatrixXf::Identity(y.size(), y.size());
        float norm_y_c = (y - this->c).norm();
        Eigen::MatrixXf outer = (y - this->c) * (y - this->c).transpose();
        return (y_dot.transpose() * (-1 / pow(norm_y_c, 3) * outer + 1 / norm_y_c * I)) / this->R;
    }; // 1 * 3
}

void CollisionAvoidance::RMP_function(void) {
    // x 1 * 1, x_dot 1 * 1, f 1 * 1, M 1 * 1
    float w, grad_w, u, grad_u, g, grad_Phi, xi, Bx_dot, M_value, f_value;
    if (x(0, 0) < 0) {
        w = 1e10;
        grad_w = 0;
    } else {
        w = 1.0 / pow(x(0, 0), 4);
        grad_w = -4.0 / pow(x(0, 0), 5);
    }

    u = epsilon + std::min(0.0f, x_dot(0, 0)) * x_dot(0, 0);
    g = w * u;
    grad_u = 2 * std::min(0.0f, x_dot(0, 0));
    grad_Phi = alpha * w * grad_w;
    xi = 0.5 * x_dot(0, 0) * x_dot(0, 0) * u * grad_w;
    M_value = g + 0.5 * x_dot(0, 0) * w * grad_u;
    M = Eigen::MatrixXf::Constant(1, 1, std::min(std::max(M_value, -1e5f), 1e5f));
    Bx_dot = eta * g * x_dot(0, 0);
    f_value = -grad_Phi - xi - Bx_dot;
    f = Eigen::MatrixXf::Constant(1, 1, std::min(std::max(f_value, -1e10f), 1e10f));
    err = x(0, 0);
}

GoalAttractorUniT::GoalAttractorUniT(std::string name, std::shared_ptr<RMPNode> parent,
                                     const Eigen::MatrixXf& y_g, float w_u, float w_l,
                                     float sigma, float alpha, float eta, float gain, float tol)
: RMPLeaf(name, parent), y_g(y_g), w_u(w_u), w_l(w_l), sigma(sigma), alpha(alpha), eta(eta), gain(gain), tol(tol) {
    psi = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        return y - this->y_g;
    }; // 3 * 1

    J = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Identity(this->y_g.size(), this->y_g.size());
    }; // 3 * 3

    J_dot = [this](const Eigen::MatrixXf& y, const Eigen::MatrixXf& y_dot) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Zero(this->y_g.size(), this->y_g.size());
    }; // 3 * 3
}

void GoalAttractorUniT::RMP_function(void) {
    // x 3 * 1, x_dot 3 * 1, f 3 * 1, M 3 * 3
    float x_norm = x.norm();
    float beta = std::exp(-std::pow(x_norm, 2) / (2 * std::pow(sigma, 2)));
    float w = (w_u - w_l) * beta + w_l;
    float s = (1 - std::exp(-2 * alpha * x_norm)) / (1 + std::exp(-2 * alpha * x_norm));
    Eigen::MatrixXf G = Eigen::MatrixXf::Identity(y_g.size(), y_g.size()) * w;
    Eigen::MatrixXf grad_Phi;
    if (x_norm > tol) {
        grad_Phi = s / x_norm * w * x * gain;
    } else {
        grad_Phi = Eigen::MatrixXf::Zero(y_g.size(), 1);
    }
    Eigen::MatrixXf Bx_dot = eta * w * x_dot;
    Eigen::MatrixXf grad_w = -beta * (w_u - w_l) / std::pow(sigma, 2) * x;

    float x_dot_norm = x_dot.norm();
    Eigen::MatrixXf xi = -0.5 * (std::pow(x_dot_norm, 2) * grad_w - 2 * (x_dot * x_dot.transpose()) * grad_w);

    M = G;
    f = -grad_Phi - Bx_dot - xi;
    err = x_norm;
}

GoalAttractorUniR::GoalAttractorUniR(std::string name, std::shared_ptr<RMPNode> parent,
                                     const Eigen::MatrixXf& y_g, float w_u, float w_l,
                                     float sigma, float alpha, float eta, float gain, float tol)
: RMPLeaf(name, parent), y_g(y_g), w_u(w_u), w_l(w_l), sigma(sigma), alpha(alpha), eta(eta), gain(gain), tol(tol) {
    psi = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        float ne = y(0, 0);
        float nd = this->y_g(0, 0);
        Eigen::Vector3f ee = {y(1, 0), y(2, 0), y(3, 0)};
        Eigen::Vector3f ed = {this->y_g(1, 0), this->y_g(2, 0), this->y_g(3, 0)};
        Eigen::Matrix3f Sed;
        Sed << 0, -ed(2), ed(1),
               ed(2), 0, -ed(0),
               -ed(1), ed(0), 0;
        Eigen::Vector3f eo = ne * ed - nd * ee - Sed * ee;
        Eigen::MatrixXf eO(3, 1);
        eO = eo;
        return eo;
    }; // 3 * 1

    J = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Identity(3, 3);
    }; // 3 * 3

    J_dot = [this](const Eigen::MatrixXf& y, const Eigen::MatrixXf& y_dot) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Zero(3, 3);
    }; // 3 * 3
}

void GoalAttractorUniR::RMP_function(void) {
    // x 3 * 1, x_dot 3 * 1, f 3 * 1, M 3 * 3
    float x_norm = x.norm();
    float beta = std::exp(-std::pow(x_norm, 2) / (2 * std::pow(sigma, 2)));
    float w = (w_u - w_l) * beta + w_l;
    float s = (1 - std::exp(-2 * alpha * x_norm)) / (1 + std::exp(-2 * alpha * x_norm));

    Eigen::MatrixXf G = Eigen::MatrixXf::Identity(x.size(), x.size()) * w;
    Eigen::MatrixXf grad_Phi;
    if (x_norm > tol) {
        grad_Phi = s / x_norm * w * x * gain;
    } else {
        grad_Phi = Eigen::MatrixXf::Zero(x.size(), 1);
    }
    Eigen::MatrixXf Bx_dot = eta * w * x_dot;
    Eigen::MatrixXf grad_w = -beta * (w_u - w_l) / std::pow(sigma, 2) * x;

    float x_dot_norm = x_dot.norm();
    Eigen::MatrixXf xi = -0.5 * (std::pow(x_dot_norm, 2) * grad_w - 2 * (x_dot * x_dot.transpose()) * grad_w);

    M = G;
    f = -grad_Phi - Bx_dot - xi;
    err = x_norm * 0.05;
}


TransitionArbitraryPoint::TransitionArbitraryPoint(std::string name, std::shared_ptr<RMPNode> parent, const Eigen::MatrixXf& point)
: RMPNode(name, parent), point(point) {
    psi = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        return this->point;
    }; // 3 * 1

    J = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Identity(this->point.size(), this->point.size());
    }; // 3 * 3

    J_dot = [this](const Eigen::MatrixXf& y, const Eigen::MatrixXf& y_dot) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Zero(this->point.size(), this->point.size());
    }; // 3 * 3
};

TransitionTaskSpace::TransitionTaskSpace(std::string name, std::shared_ptr<RMPNode> parent, const int& link, const float& length)
: RMPNode(name, parent), link(link), length(length) {
    k = std::shared_ptr<KinematicsR>(new KinematicsR(link, length));
    psi = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        Eigen::MatrixXf pos = this->k->getPosFK(y);
        return pos;
    }; // 3 * 1

    J = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        Eigen::MatrixXf Jpos = this->k->getPosJ(y);
        return Jpos;
    }; // 3 * 3 or 3 * 6

    J_dot = [this](const Eigen::MatrixXf& y, const Eigen::MatrixXf& y_dot) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Zero(3, y.size());
    }; // 3 * 3 or 3 * 6
};

RotationTaskSpace::RotationTaskSpace(std::string name, std::shared_ptr<RMPNode> parent, const int& link, const float& length)
: RMPNode(name, parent), link(link), length(length) {
    k = std::shared_ptr<KinematicsR>(new KinematicsR(link, length));
    psi = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        Eigen::MatrixXf Q = this->k->getOriFK(y);
        return Q;
    }; // 4 * 1

    J = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        Eigen::MatrixXf Jrot = this->k->getOriJ(y);
        return Jrot;
    }; // 3 * 3 or 3 * 6

    J_dot = [this](const Eigen::MatrixXf& y, const Eigen::MatrixXf& y_dot) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Zero(3, y.size());
    }; // 3 * 3 or 3 * 6
};


void RotationTaskSpace::pullback() {
    std::cout << name << ": pullback" << std::endl;
    for (auto& child : children) {
        child->pullback();
    }
    if (verbose) {
        std::cout << name << ": pullback" << std::endl;
    }
    Eigen::MatrixXf f_total = Eigen::MatrixXf::Zero(3, 1);
    Eigen::MatrixXf M_total = Eigen::MatrixXf::Zero(3, 3);
    for (auto& child : children) {
        Eigen::MatrixXf J_child = child->J(x);
        Eigen::MatrixXf J_dot_child = child->J_dot(x, x_dot);
        if (child->f.size() > 0 && child->M.size() > 0) {
            Eigen::MatrixXf f_child = J_child.transpose() * child->f;
            // if (f_child.rows() != x.rows()) {
            //     Eigen::MatrixXf temp = Eigen::MatrixXf::Zero(x.rows(), f_child.cols());
            //     temp.topRows(f_child.rows()) = f_child;
            //     f_child = temp;
            // }
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
