//
// Created by symc on 2025/4/11.
//

#ifndef MULTI_AGENT_AGENT_H
#define MULTI_AGENT_AGENT_H

#include "Eigen/Dense"

class Agent {
public:
    Eigen::MatrixXd A_, B_, C_; // system matrices
    Eigen::VectorXd x_; // state
    double y_; // output
    double z_, v_, z_dot_, v_dot_, z_dot_new_, v_dot_new_; // optimal signal generator
    double z_hat_, v_hat_; // for event-triggered communication
    double u_, u_new_, u_hat_; // control input

    Eigen::RowVectorXd K1_; // ki1 controller gain
    double K2_; // ki2 controller gain

    // time
    double last_control_time_;
    double last_comm_time_;

    // parameters
    double alpha_, beta_;

    // event-triggered parameters
    double c0_, c1_, gamma_; // controller trigger
    double c0_comm_, c1_comm_, gamma_comm_; // communication trigger

    std::function<double(double)> fi_grad_; // gradient of the potential function

    Agent(int dim);

    void SetSystem(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C);
    void SetController(const Eigen::RowVectorXd& K1, double K2);
    void SetOptimizerParams(double alpha, double beta);
    void SetEventTriggerParams(double c0, double c1, double gamma, double c0c, double c1c, double gammac);

    void UpdateOutput(); // y = C * x
    void UpdateController(double t); // u = K1 * x + K2 * z
    void UpdateDynamics(double dt); // x_dot = A * x + B * u
    void UpdateSignalGenerator(double t, double dt, const std::vector<Agent>& neighbors); // z_dot, v_dot

    bool ShouldTriggerControl(double t);
    bool ShouldTriggerCommunication(double t);

};

#endif //MULTI_AGENT_AGENT_H
