//
// Created by symc on 2025/4/11.
//
#include <iostream>
#include <iomanip>
#include "include/agent.h"

Agent::Agent(int dim) {
    x_ = Eigen::VectorXd::Zero(dim);
    y_ = 0;
    z_ = v_ = z_dot_ = v_dot_ = z_dot_new_ = v_dot_new_ = 0;
    z_hat_ =  v_hat_ = 0;
    u_ = u_new_ = 0;
    u_hat_ = 0;
    last_control_time_ = last_comm_time_ = 0;
}

void Agent::SetSystem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &C) {
    A_ = A;
    B_ = B;
    C_ = C;
}

void Agent::SetController(const Eigen::RowVectorXd &K1, double K2) {
    K1_ = K1;
    K2_ = K2;
}

void Agent::SetOptimizerParams(double alpha, double beta) {
    alpha_ = alpha;
    beta_ = beta;
}
void Agent::SetEventTriggerParams(double c0, double c1, double gamma, double c0c, double c1c, double gammac) {
    c0_ = c0;
    c1_ = c1;
    gamma_ = gamma;
    c0_comm_ = c0c;
    c1_comm_ = c1c;
    gamma_comm_ = gammac;
}
void Agent::UpdateOutput() {
    y_ = (C_ * x_)(0, 0);
}
void Agent::UpdateController(double t) {
    u_new_ = K1_ * x_ + K2_ * z_;
    if(ShouldTriggerControl(t)) {
//        { // test
//            std::cout<<"Control Triggered at t = "<<t<<std::endl;
//        }

        u_ = u_new_;
        u_hat_ = u_;
        last_control_time_ = t;
    }
}
void Agent::UpdateDynamics(double dt) {
    auto rk4 = [&](const Eigen::VectorXd& x_in) -> Eigen::VectorXd {
        return A_ * x_in + B_ * u_;
    };
    Eigen::VectorXd k1 = rk4(x_);
    Eigen::VectorXd k2 = rk4(x_ + 0.5 * dt * k1);
    Eigen::VectorXd k3 = rk4(x_ + 0.5 * dt * k2);
    Eigen::VectorXd k4 = rk4(x_ + dt * k3);
    x_ = x_ + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
    UpdateOutput();
}
void Agent::UpdateSignalGenerator(double t, double dt, const std::vector<Agent> &neighbors) {
    auto rk4 = [&](const double z_in, const double v_in){
        double grad = fi_grad_(z_in);
        double sum_z = 0, sum_v = 0;
        for(const auto& neighbor : neighbors) {
            sum_z += (z_hat_ - neighbor.z_hat_);
            sum_v += (v_hat_ - neighbor.v_hat_);
        }
        double z_dot = -alpha_ * grad - beta_ * sum_z - sum_v;
        double v_dot = alpha_ * beta_ * sum_z;
        return std::make_pair(z_dot, v_dot);
    };
    auto [k1_z, k1_v] = rk4(z_, v_);
    auto [k2_z, k2_v] = rk4(z_ + 0.5 * dt * k1_z, v_ + 0.5 * dt * k1_v);
    auto [k3_z, k3_v] = rk4(z_ + 0.5 * dt * k2_z, v_ + 0.5 * dt * k2_v);
    auto [k4_z, k4_v] = rk4(z_ + dt * k3_z, v_ + dt * k3_v);
//    { // test}
//        std::cout<<"k1_z = "<<k1_z<<", k1_v = "<<k1_v<<std::endl;
//        std::cout<<"k2_z = "<<k2_z<<", k2_v = "<<k2_v<<std::endl;
//        std::cout<<"k3_z = "<<k3_z<<", k3_v = "<<k3_v<<std::endl;
//        std::cout<<"k4_z = "<<k4_z<<", k4_v = "<<k4_v<<std::endl;
//    }
    z_dot_new_ = (1.0 / 6) * (k1_z + 2 * k2_z + 2 * k3_z + k4_z);

    v_dot_new_ = (1.0 / 6) * (k1_v + 2 * k2_v + 2 * k3_v + k4_v);

    if(ShouldTriggerCommunication(t)) {
//        { // test
//            std::cout<<"Communication Triggered at t = "<<t<<std::endl;
//        }
        z_dot_ = z_dot_new_;
        v_dot_ = v_dot_new_;
        z_hat_ = z_;
        v_hat_ = v_;
        last_comm_time_ = t;
    }
    z_ += z_dot_ * dt;
    v_ += v_dot_ * dt;
}

bool Agent::ShouldTriggerControl(double t) {
    if(t == 0) return true; // trigger at t = 0
    return std::abs(u_new_ - u_hat_) >= c0_ + c1_ * std::exp(-gamma_ * t);
}
bool Agent::ShouldTriggerCommunication(double t) {
    if(t == 0) return true; // trigger at t = 0
    double dz = z_ - z_hat_;
    double dv = v_ - v_hat_;

    double threshold = c0_comm_ + c1_comm_ * std::exp(-gamma_comm_ * t);
    return std::sqrt(dz * dz + dv * dv) >= threshold;
}
