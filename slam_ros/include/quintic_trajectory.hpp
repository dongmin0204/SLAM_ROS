#ifndef QUINTIC_TRAJECTORY_HPP
#define QUINTIC_TRAJECTORY_HPP

#include <Eigen/Dense>
#include <vector>

class QuinticTrajectory {
public:
    struct State {
        double x, y, theta;
        double vx, vy, omega;
    };

    QuinticTrajectory(double dt = 0.05);
    
    // Generate trajectory from start to goal
    std::vector<State> generate(const State& start, const State& goal, double T);
    
    // Sample trajectory at specific time
    State sample(const std::vector<State>& trajectory, double t);

private:
    double dt_;  // Fixed time step (0.05s)
    
    // Calculate quintic polynomial coefficients
    Eigen::VectorXd calculateCoefficients(double start, double goal, double T);
    
    // Convert coefficients to state
    State coefficientsToState(const Eigen::VectorXd& coeffs_x,
                            const Eigen::VectorXd& coeffs_y,
                            double t);
};

#endif // QUINTIC_TRAJECTORY_HPP 