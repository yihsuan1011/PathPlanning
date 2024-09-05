# include "ActualNode.h"
# include <typeinfo>
# include <fstream>

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
        Eigen::MatrixXf j_dot = (y_dot.transpose() * (-1 / pow(norm_y_c, 3) * outer + 1 / norm_y_c * I)) / this->R;
        // return (y_dot.transpose() * (-1 / pow(norm_y_c, 3) * outer + 1 / norm_y_c * I)) / this->R;
        return j_dot;
    }; // 1 * 3
}

void CollisionAvoidance::RMP_function(void) {
    // x 1 * 1, x_dot 1 * 1, f 1 * 1, M 1 * 1
    float w, grad_w, u, grad_u, g, grad_Phi, xi, Bx_dot, M_value, f_value;
    if (x(0, 0) < 0) {
        std::cout << "\033[31;1m" << name << ": x < 0 !!!!!!!!!!!!!!!!!!!!!!" << "\033[0m" << std::endl;
        w = 1e10;
        grad_w = 0;
    } else {
        w = 1.0 / pow(x(0, 0), 4);
        grad_w = -4.0 / pow(x(0, 0), 5);
    }
    if (x_dot(0, 0) < 0)
        std::cout << "\033[36;1m" << name << ": x_dot < 0 !!!!!!!!!!!!!!!!!!!!!!" << "\033[0m" << std::endl;
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
        // std::cout << "y = " << y.transpose() << std::endl;
        // std::cout << "y_g = " << this->y_g.transpose() << std::endl;
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
    float x_norm, beta, w, s, x_dot_norm;
    Eigen::MatrixXf G, grad_Phi, Bx_dot, grad_w, xi;

    x_norm = x.norm();
    beta = std::exp(-std::pow(x_norm, 2) / (2 * std::pow(sigma, 2)));
    w = (w_u - w_l) * beta + w_l;
    s = (1 - std::exp(-2 * alpha * x_norm)) / (1 + std::exp(-2 * alpha * x_norm));
    G = Eigen::MatrixXf::Identity(y_g.size(), y_g.size()) * w;
    if (x_norm > tol) {
        grad_Phi = s / x_norm * w * x * gain;
    } else {
        grad_Phi = Eigen::MatrixXf::Zero(y_g.size(), 1);
    }
    Bx_dot = eta * w * x_dot;
    grad_w = -beta * (w_u - w_l) / std::pow(sigma, 2) * x;

    x_dot_norm = x_dot.norm();
    xi = -0.5 * (std::pow(x_dot_norm, 2) * grad_w - 2 * (x_dot * x_dot.transpose()) * grad_w);

    M = G;
    f = -grad_Phi - Bx_dot - xi;
    // std::cout << "error_T = " << x_norm << std::endl;
    err = x_norm;

    std::ofstream file;
    // file.open("GoalError.txt");
    file.open("/home/aiRobots/Calvin/src/calvin/PathPlanning/src/GoalError.txt", std::ios::app);

    if (file.is_open()) {
        file << x_norm << " " << std::endl;
    }
    file.close();
}

GoalAttractorUniR::GoalAttractorUniR(std::string name, std::shared_ptr<RMPNode> parent, const int& mode,
                                     const Eigen::MatrixXf& y_g, float w_u, float w_l,
                                     float sigma, float alpha, float eta, float gain, float tol)
: RMPLeaf(name, parent), mode(mode), y_g(y_g), w_u(w_u), w_l(w_l), sigma(sigma), alpha(alpha), eta(eta), gain(gain), tol(tol) {
    psi = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        // std::cout << "y = " << y.transpose() << std::endl;
        // std::cout << "y_g = " << this->y_g.transpose() << std::endl;

        // // 3 * 1 from Robotics
        // float ne = y(0, 0);
        // float nd = this->y_g(0, 0);
        // Eigen::Vector3f ee = {y(1, 0), y(2, 0), y(3, 0)};
        // Eigen::Vector3f ed = {this->y_g(1, 0), this->y_g(2, 0), this->y_g(3, 0)};
        // Eigen::Matrix3f Sed;
        // Sed << 0, -ed(2), ed(1),
        //        ed(2), 0, -ed(0),
        //        -ed(1), ed(0), 0;
        // Eigen::Vector3f eo = ne * ed - nd * ee - Sed * ee;
        // Eigen::MatrixXf eO(3, 1);
        // eO = eo;
        // std::cout << "eO = " << eO.transpose() << std::endl;
        // return eo;

        // 4 * 1 from StackExchange
        float qw = y(0, 0) * this->y_g(0, 0) + y(1, 0) * this->y_g(1, 0) + y(2, 0) * this->y_g(2, 0) + y(3, 0) * this->y_g(3, 0);
        float qx = y(0, 0) * this->y_g(1, 0) - y(1, 0) * this->y_g(0, 0) - y(2, 0) * this->y_g(3, 0) - y(3, 0) * this->y_g(2, 0);
        float qy = y(0, 0) * this->y_g(2, 0) - y(2, 0) * this->y_g(0, 0) - y(1, 0) * this->y_g(3, 0) + y(3, 0) * this->y_g(1, 0);
        float qz = y(0, 0) * this->y_g(3, 0) - y(3, 0) * this->y_g(0, 0) + y(1, 0) * this->y_g(2, 0) - y(2, 0) * this->y_g(1, 0);
        
        // // qg * qc^(-1)
        // Eigen::Quaternionf qg(this->y_g(0, 0), this->y_g(1, 0), this->y_g(2, 0), this->y_g(3, 0));
        // Eigen::Quaternionf qc(y(0, 0), -y(1, 0), -y(2, 0), -y(3, 0));
        // Eigen::Quaternionf qe = qg * qc;

        // // euler
        // Eigen::Matrix3f Re = qe.toRotationMatrix();
        // Eigen::Vector3f euler;
        // euler(2) = atan2(Re(2, 1), Re(2, 2));
        // euler(1) = atan2(-Re(2, 0), sqrt(Re(2, 1) * Re(2, 1) + Re(2, 2) * Re(2, 2)));
        // euler(0) = atan2(Re(1, 0), Re(0, 0));
        
        if (this->mode == 1) {
            Eigen::MatrixXf eO(1, 1);
            eO << qx;
            return eO;
        } else if (this->mode == 2) {
            Eigen::MatrixXf eO(1, 1);
            eO << qy;
            return eO;
        } else if (this->mode == 3) {
            Eigen::MatrixXf eO(1, 1);
            eO << qz;
            return eO;
        } else {
            Eigen::MatrixXf eO(3, 1);
            eO << qx, qy, qz;
            return eO;
        }
        // std::cout << "eO = " << eO.transpose() << std::endl;
        // return eO;
    };

    J = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        if (this->mode == 0)
            return Eigen::MatrixXf::Identity(3, 3);
        else
            return Eigen::MatrixXf::Identity(1, 1);
    }; // 3 * 3

    J_dot = [this](const Eigen::MatrixXf& y, const Eigen::MatrixXf& y_dot) -> Eigen::MatrixXf {
        if (this->mode == 0)
            return Eigen::MatrixXf::Zero(3, 3);
        else
            return Eigen::MatrixXf::Zero(1, 1);
    }; // 3 * 3
}

void GoalAttractorUniR::RMP_function(void) {
    // x 3 * 1, x_dot 3 * 1, f 3 * 1, M 3 * 3
    float x_norm, beta, w, s, x_dot_norm;
    Eigen::MatrixXf G, grad_Phi, Bx_dot, grad_w, xi;

    // float x_norm = x.norm();
    // Eigen::MatrixXf xe = x.block(1, 0, 3, 1);
    // float x_norm = atan2(xe.norm(), x(0, 0));
    x_norm = -log(1 - x.norm());
    // x_norm = x.norm();
    beta = std::exp(-std::pow(x_norm, 2) / (2 * std::pow(sigma, 2)));
    w = (w_u - w_l) * beta + w_l;
    s = (1 - std::exp(-2 * alpha * x_norm)) / (1 + std::exp(-2 * alpha * x_norm));
    G = Eigen::MatrixXf::Identity(x.size(), x.size()) * w;
    if (x_norm > tol) {
        grad_Phi = s / x_norm * w * x * gain;
    } else {
        grad_Phi = Eigen::MatrixXf::Zero(x.size(), 1);
    }
    Bx_dot = eta * w * x_dot;
    grad_w = -beta * (w_u - w_l) / std::pow(sigma, 2) * x;

    x_dot_norm = x_dot.norm();
    xi = -0.5 * (std::pow(x_dot_norm, 2) * grad_w - 2 * (x_dot * x_dot.transpose()) * grad_w);

    M = G;
    f = -grad_Phi - Bx_dot - xi;
    std::cout << "error_O = " << x_norm << std::endl;
    err = x_norm * 0.05;

    std::ofstream file;
    file.open("/home/aiRobots/Calvin/src/calvin/PathPlanning/src/GoalError.txt", std::ios::app);
    if (file.is_open()) {
        file << x.norm() << std::endl;
    }
    file.close();

}

Damper::Damper(std::string name, std::shared_ptr<RMPNode> parent, float w, float eta)
: RMPLeaf(name, parent), w(w), eta(eta) {
    psi = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        return y;
    };

    J = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Identity(y.size(), y.size());
    };

    J_dot = [this](const Eigen::MatrixXf& y, const Eigen::MatrixXf& y_dot) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Zero(y.size(), y.size());
    };
}

void Damper::RMP_function(void) {
    M = w * Eigen::MatrixXf::Identity(x.size(), x.size());
    f = -eta * w * x_dot;
}

JointLimit::JointLimit(std::string name, std::shared_ptr<RMPNode> parent, float lam, float sigma, float nu_p, float nu_d)
: RMPLeaf(name, parent), lam(lam), sigma(sigma), nu_p(nu_p), nu_d(nu_d) {
    // l_l 3 * 1, l_u 3 * 1, x_0 3 * 1
    // y 3 * 1
    // y_dot 3 * 1

    psi = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        return y;
    }; // 3 * 1

    J = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Identity(y.size(), y.size());
    }; // 3 * 3

    J_dot = [this](const Eigen::MatrixXf& y, const Eigen::MatrixXf& y_dot) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Zero(y.size(), y.size());
    }; // 3 * 3

    l_l = Eigen::MatrixXf(6, 1);
    l_l << -90, -180, -180, -180, -130, -180;
    l_l = l_l / 180 * M_PI;
    l_u = Eigen::MatrixXf(6, 1);
    l_u << 90, 180, 20, 180, 130, 180;
    l_u = l_u / 180 * M_PI;
    x_0 = Eigen::MatrixXf::Zero(6, 1);
}

Eigen::MatrixXf JointLimit::d(const Eigen::MatrixXf& q, const Eigen::MatrixXf& qd) {
    Eigen::MatrixXf s, d, alpha_l, alpha_u, result;
    s = (q - l_l).array() / (l_u - l_l).array();
    d = 4 * s.array() * (1 - s.array());
    alpha_l = 1 - (-qd.array().min(0).square() / (2 * sigma * sigma)).exp();
    alpha_u = 1 - (-qd.array().max(0).square() / (2 * sigma * sigma)).exp();
    result = (s.array() * (alpha_u.array() * d.array() + (1 - alpha_u.array())) + 
             (1 - s.array()) * (alpha_l.array() * d.array() + (1 - alpha_l.array()))).array().pow(-2);
    return result;
}

Eigen::MatrixXf JointLimit::grad_d(const Eigen::MatrixXf& q, const Eigen::MatrixXf& qd) {
    Eigen::MatrixXf s, d, alpha_l, alpha_u, b, ds_dq, dd_dq, db_dq, result;
    s = (q - l_l).array() / (l_u - l_l).array();
    d = 4 * s.array() * (1 - s.array());
    alpha_l = 1 - (-qd.array().min(0).square() / (2 * sigma * sigma)).exp();
    alpha_u = 1 - (-qd.array().max(0).square() / (2 * sigma * sigma)).exp();
    b =  s.array() * (alpha_u.array() * d.array() + (1 - alpha_u.array())) + 
         (1 - s.array()) * (alpha_l.array() * d.array() + (1 - alpha_l.array()));
    ds_dq = (1 / (l_u - l_l).array()).array();
    dd_dq = -4 * (s.array() - 1) * ds_dq.array() - 4 * s.array() * ds_dq.array();
    db_dq = (alpha_u.array() * d.array() - alpha_u.array() + 1) * ds_dq.array() - 
            (alpha_l.array() * d.array() - alpha_l.array() + 1) * ds_dq.array() + 
            alpha_u.array() * s.array() * dd_dq.array() - 
            alpha_l.array() * (s.array() - 1) * dd_dq.array();
    result = -2 * db_dq.array() / b.array().cube();
    return result;
}

void JointLimit::RMP_function(void) {
    Eigen::MatrixXf d_val, grad_d_val, xi;
    d_val = d(x, x_dot);
    grad_d_val = grad_d(x, x_dot);
    M = lam * d_val.asDiagonal();
    xi = 0.5 * grad_d_val.array() * x_dot.array().square();
    f = M * (nu_p * (x_0 - x) - nu_d * x_dot) - xi;
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
        // Eigen::MatrixXf Jtest = Jpos * Jpos.transpose();
        // float det = Jtest.determinant();
        // std::cout << "Tdet = " << det << std::endl;
        return Jpos;
    }; // 3 * 3 or 3 * 6

    J_dot = [this](const Eigen::MatrixXf& y, const Eigen::MatrixXf& y_dot) -> Eigen::MatrixXf {
        return Eigen::MatrixXf::Zero(3, y.size());
    }; // 3 * 3 or 3 * 6
};


void TransitionTaskSpace::pullback() {
    for (auto& child : children) {
        child->pullback();
    }
    if (verbose) {
        std::cout << name << ": pullback" << std::endl;
    }
    float mu = 0.2;
    Eigen::MatrixXf f_total = Eigen::MatrixXf::Zero(3, 1);
    Eigen::MatrixXf M_total = Eigen::MatrixXf::Zero(3, 3);
    for (auto& child : children) {
        Eigen::MatrixXf J_child = child->J(x);
        Eigen::MatrixXf J_damped = J_child.transpose() * (J_child * J_child.transpose() + mu * Eigen::MatrixXf::Identity(3, 3)).inverse();
        J_child = J_damped.transpose();
        if (child->f.size() > 0 && child->M.size() > 0) {
            Eigen::MatrixXf f_child = J_child.transpose() * child->f;
            f_total += f_child;
            Eigen::MatrixXf M_child = J_child.transpose() * child->M * J_child;
            M_total += M_child;
        }
    }
    f = f_total;
    M = M_total;
}

RotationTaskSpace::RotationTaskSpace(std::string name, std::shared_ptr<RMPNode> parent, const int& mode, const int& link, const float& length)
: RMPNode(name, parent), mode(mode), link(link), length(length) {
    k = std::shared_ptr<KinematicsR>(new KinematicsR(link, length));
    psi = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        Eigen::MatrixXf Q = this->k->getOriFK(y);
        return Q;
    }; // 4 * 1

    J = [this](const Eigen::MatrixXf& y) -> Eigen::MatrixXf {
        Eigen::MatrixXf Jrot = this->k->getOriJ(y);
        if (this->mode == 1) {
            return Jrot.block(0, 0, 1, 6);
        } else if (this->mode == 2) {
            return Jrot.block(1, 0, 1, 6);
        } else if (this->mode == 3) {
            return Jrot.block(2, 0, 1, 6);
        } else {
            // Eigen::MatrixXf Jtest = Jrot * Jrot.transpose();
            // float det = Jtest.determinant();
            // std::cout << "Rdet = " << det << std::endl;
            return Jrot;
        }
        // return Jrot;
    }; // 3 * 3 or 3 * 6

    J_dot = [this](const Eigen::MatrixXf& y, const Eigen::MatrixXf& y_dot) -> Eigen::MatrixXf {
        if (this->mode == 0) {
            return Eigen::MatrixXf::Zero(3, y.size()); 
        } else {
            return Eigen::MatrixXf::Zero(1, y.size());
        }
    }; // 3 * 3 or 3 * 6
};


void RotationTaskSpace::pullback() {
    for (auto& child : children) {
        child->pullback();
    }
    if (verbose) {
        std::cout << name << ": pullback" << std::endl;
    }
    float mu = 0.2;
    Eigen::MatrixXf f_total;
    Eigen::MatrixXf M_total;
    if (mode == 0) {
        f_total = Eigen::MatrixXf::Zero(3, 1);
        M_total = Eigen::MatrixXf::Zero(3, 3);
    } else {
        f_total = Eigen::MatrixXf::Zero(1, 1);
        M_total = Eigen::MatrixXf::Zero(1, 1);
    }
    for (auto& child : children) {
        Eigen::MatrixXf J_child = child->J(x);
        Eigen::MatrixXf J_damped = J_child.transpose() * (J_child * J_child.transpose() + mu * Eigen::MatrixXf::Identity(3, 3)).inverse();
        J_child = J_damped.transpose();
        if (child->f.size() > 0 && child->M.size() > 0) {
            Eigen::MatrixXf f_child = J_child.transpose() * child->f;
            f_total += f_child;
            Eigen::MatrixXf M_child = J_child.transpose() * child->M * J_child;
            M_total += M_child;
        }
    }
    f = f_total;
    M = M_total;
}
