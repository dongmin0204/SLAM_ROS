#ifndef MPC_SOLVER_HPP
#define MPC_SOLVER_HPP

#include <Eigen/Dense>
#include <vector>
#include <osqp_eigen/osqp_eigen.h>
#include "kinematic_model.hpp"

class MpcSolver {
public:
    struct MpcConfig {
        int prediction_horizon;  // N
        double dt;              // Time step
        Eigen::Matrix3d Q;      // State cost matrix
        Eigen::Matrix3d Rd;     // Input rate cost matrix
        double v_max;           // Maximum linear velocity
        double omega_max;       // Maximum angular velocity
    };

    MpcSolver(const MpcConfig& config);
    
    // Compute control input
    Eigen::Vector3d computeControl(
        const Eigen::Vector3d& current_state,
        const std::vector<Eigen::Vector3d>& reference_trajectory);
    
    // Update system matrices
    void updateSystemMatrices(const Eigen::Vector3d& current_state);

private:
    MpcConfig config_;
    KinematicModel kinematic_model_;
    
    // System matrices
    Eigen::Matrix3d A_;  // State transition matrix
    Eigen::Matrix3d B_;  // Input matrix
    
    // QP problem matrices
    Eigen::SparseMatrix<double> P_;  // Quadratic cost matrix
    Eigen::VectorXd q_;             // Linear cost vector
    Eigen::SparseMatrix<double> A_constraint_;  // Constraint matrix
    Eigen::VectorXd l_constraint_;  // Lower bounds
    Eigen::VectorXd u_constraint_;  // Upper bounds
    
    // OSQP solver
    OsqpEigen::Solver solver_;
    
    // Initialize QP problem
    void initializeQP();
    
    // Update QP constraints
    void updateConstraints(const Eigen::Vector3d& current_state);
};

#endif // MPC_SOLVER_HPP 