#ifndef KINEMATIC_MODEL_HPP
#define KINEMATIC_MODEL_HPP

#include <Eigen/Dense>

class KinematicModel {
public:
    // Constants
    static constexpr double WHEEL_RADIUS = 0.0375;  // m
    static constexpr double LxLy = 0.135;          // m (half of robot length)

    KinematicModel();
    
    // Forward kinematics: wheel velocities to body velocities
    Eigen::Vector3d wheelToBodyVelocities(const Eigen::Vector4d& wheel_velocities);
    
    // Inverse kinematics: body velocities to wheel velocities
    Eigen::Vector4d bodyToWheelVelocities(const Eigen::Vector3d& body_velocities);
    
    // Get wheel Jacobian matrix
    Eigen::Matrix<double, 4, 3> getWheelJacobian();
    
    // Get inverse wheel Jacobian matrix
    Eigen::Matrix<double, 3, 4> getInverseWheelJacobian();

private:
    // Jacobian matrices
    Eigen::Matrix<double, 4, 3> J_;  // Wheel Jacobian
    Eigen::Matrix<double, 3, 4> J_inv_;  // Inverse Wheel Jacobian
    
    // Initialize Jacobian matrices
    void initializeJacobians();
};

#endif // KINEMATIC_MODEL_HPP 