//
// Created by symc on 2025/4/12.
//
#include <iostream>
#include <fstream>
#include <vector>
#include "agent.h"

int main() {
    double dt = 0.01;
    double T = 40.0;

    Agent agent(3); // 创建一个Agent对象，维度为3
    Eigen::MatrixXd A(3, 3); A << 0, 1, 0, -1, 0, 1, 2, 0, 1;
    Eigen::MatrixXd B(3, 1); B << 0, 1, 1;
    Eigen::MatrixXd C(1, 3); C << 0, 1, 0;
    agent.SetSystem(A, B, C);
    Eigen::RowVectorXd K1(3); K1 << -2.7331, -2.7331, -3.5835;
    double K2 = 3.3166;
    agent.SetController(K1, K2);
    agent.SetOptimizerParams(1.0, 10.0);
    agent.fi_grad_ = [](double s) -> double {
        double num = -0.1 * exp(-0.1 * s) + 0.3 *exp(0.3 * s);
        double den = exp(-0.1 * s) + exp(0.3 * s);
        return num / den + 2 * s;
    };
    agent.SetEventTriggerParams(0.0, 5.0, 0.5, 0.0, 5.0, 0.1);
    agent.x_ = Eigen::VectorXd::Random(3) * 5; // 随机初始状态
    agent.UpdateOutput(); // 更新输出

    // 输出结果文件
    std::ofstream file("agent1_output.csv");
    file << "time,y,z,u\n";

    // 仿真主循环
    for (double t = 0; t <= T; t += dt) {
        agent.UpdateController(t);
//        agent.UpdateSignalGenerator(t, dt, std::vector<Agent>{}); // 没有邻居
        double y_ref = 1.0;
        double gamma = 0.5;
        double dz = -gamma * (agent.z_ - y_ref);
        agent.z_ += dz * dt;
        agent.UpdateDynamics(dt);

        // 输出数据
        file << t << "," << agent.y_ << "," << agent.z_ << "," << agent.u_ << "\n";
        std::cout<<"t = " << t << ", y = " << agent.y_ << ", z = " << agent.z_ << ", u = " << agent.u_ << std::endl;
    }

    file.close();
    std::cout << "data saved to agent1_output.csv" << std::endl;
    return 0;
}