

#define MY_ALPHA_ (1.0)
#define MY_BETA_ (10.0)
#define MY_C1_ (2.5)

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "agent.h"

void Agent1Setup(Agent& agent);
void Agent2Setup(Agent& agent);
void Agent3Setup(Agent& agent);
void Agent4Setup(Agent& agent);

// graph
std::vector<std::vector<int>> adjacency = {
        {2},    // agent1 ← agent3
        {0},    // agent2 ← agent1
        {1, 3}, // agent3 ← agent2, agent4
        {2}     // agent4 ← agent3
};

int main(){
    const int N = 4; // number of agents
    const double dt = 0.01; // time step
    const double T = 100.0; // total time

    std::vector<Agent> agents;

    // Create agents
    for(int i = 0; i < N; ++i) {
        if(i == 0) {
            agents.emplace_back(1); // first agent
        }else if(i == 1 || i == 3){
            agents.emplace_back(2); // second and fourth agents
        }else{
            agents.emplace_back(3); // third agent
        }
    }

    // Set system dynamics for each agent
    Agent1Setup(agents[0]);
    Agent2Setup(agents[1]);
    Agent3Setup(agents[2]);
    Agent4Setup(agents[3]);

    std::ofstream file("output.csv");
    file << "t,y1,y2,y3,y4\n";

    // Simulation loop
    for(double t = 0; t <= T; t += dt) {
        for(int i = 0; i < N; ++i) {
            std::vector<Agent> neighbors;
            for(int j : adjacency[i]) {
                neighbors.push_back(agents[j]);
            }
            agents[i].UpdateSignalGenerator(t, dt, neighbors); // pass all agents as neighbors
        }
        for(auto& agent : agents) {
            agent.UpdateController(t);
            agent.UpdateDynamics(dt);
        }
        // Output data
        file << t;
        for(int i = 0; i < N; ++i) {
            file << "," << agents[i].y_;
            file << "," << agents[i].z_;
        }
        file << "\n";
    }
    file.close();
    std::cout << "Simulation finished, results saved to output.csv" << std::endl;

    return 0;
}
void Agent1Setup(Agent& agent){
    Eigen::MatrixXd A(1, 1); A << 1;
    Eigen::MatrixXd B(1, 1); B << 1;
    Eigen::MatrixXd C(1, 1); C << 1;
    agent.SetSystem(A, B, C);
    Eigen::RowVectorXd K1(1); K1 << -2.4142;
    double K2 = 1.4142;
    agent.SetController(K1, K2);
    agent.SetOptimizerParams(MY_ALPHA_, MY_BETA_);
    agent.fi_grad_ = [](double s) -> double { return s - 2; };
    agent.SetEventTriggerParams(0.0, MY_C1_, 0.5, 0.0, MY_C1_, 0.1);
    // random initial state from -5 to 5
    agent.x_ = Eigen::VectorXd::Random(1) * 5;
    agent.UpdateOutput();
}

void Agent2Setup(Agent& agent){
    Eigen::MatrixXd A(2, 2); A << 0, 1, -1, 0;
    Eigen::MatrixXd B(2, 1); B << 0, 1;
    Eigen::MatrixXd C(1, 2); C << 1, 0;
    agent.SetSystem(A, B, C);
    Eigen::RowVectorXd K1(2); K1 << -0.4142, -1.3522;
    double K2 = 1.4142;
    agent.SetController(K1, K2);
    agent.SetOptimizerParams(MY_ALPHA_, MY_BETA_);
    agent.fi_grad_ = [](double s) -> double { return 2 * s * log(1 + s * s) + (2 * s * s * s) / (1 + s * s) + 2 * (s + 1); };
    agent.SetEventTriggerParams(0.0, MY_C1_, 0.5, 0.0, MY_C1_, 0.1);
    agent.x_ = Eigen::VectorXd::Random(2) * 5;
    agent.UpdateOutput();
}

void Agent3Setup(Agent& agent){
    Eigen::MatrixXd A(3, 3); A << 0, 1, 0, -1, 0, 1, 2, 0, 1;
    Eigen::MatrixXd B(3, 1); B << 0, 1, 1;
    Eigen::MatrixXd C(1, 3); C << 0, 1, 0;
    agent.SetSystem(A, B, C);
    Eigen::RowVectorXd K1(3); K1 << -2.7331, -2.7331, -3.5835;
    double K2 = 3.3166;
    agent.SetController(K1, K2);
    agent.SetOptimizerParams(MY_ALPHA_, MY_BETA_);
    agent.fi_grad_ = [](double s) -> double {
        double num = -0.1 * exp(-0.1 * s) + 0.3 *exp(0.3 * s);
        double den = exp(-0.1 * s) + exp(0.3 * s);
        return num / den + 2 * s;
    };
    agent.SetEventTriggerParams(0.0, MY_C1_, 0.5, 0.0, MY_C1_, 0.1);
    agent.x_ = Eigen::VectorXd::Random(3) * 5;
    agent.UpdateOutput();
}

void Agent4Setup(Agent& agent){
    Eigen::MatrixXd A(2, 2); A << 0, 1, 0, 0;
    Eigen::MatrixXd B(2, 1); B << 0, 1;
    Eigen::MatrixXd C(1, 2); C << 1, 0;
    agent.SetSystem(A, B, C);
    Eigen::RowVectorXd K1(2); K1 << -1.0, -1.7321;
    double K2 = 1.0;
    agent.SetController(K1, K2);
    agent.SetOptimizerParams(MY_ALPHA_, MY_BETA_);
    agent.fi_grad_ = [](double s) {
        return (2*s/25.0)*(sqrt(s*s + 1) - s*s / pow(s*s + 1, 1.5)) + 2 * (s - 3);
    };
    agent.SetEventTriggerParams(0.0, MY_C1_, 0.5, 0.0, MY_C1_, 0.1);
    agent.x_ = Eigen::VectorXd::Random(2) * 5;
    agent.UpdateOutput();
}