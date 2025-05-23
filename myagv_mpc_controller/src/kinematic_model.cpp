#include "kinematic_model.hpp"

MecanumKinematics::MecanumKinematics(double r, double lxly)
  : r_(r), lxly_(lxly)
{
  // Jacobian 행렬 초기화 (4×3)
  W_ << 1, -1, -lxly_,
        1,  1,  lxly_,
        1,  1, -lxly_,
        1, -1,  lxly_;
  W_ /= r_;
}

Eigen::Vector4d MecanumKinematics::bodyToWheel(const Eigen::Vector3d& twist) const
{
  return W_ * twist;
}

Eigen::Vector3d MecanumKinematics::wheelToBody(const Eigen::Vector4d& w) const
{
  // W^T (W W^T)^(-1) w
  Eigen::Matrix<double,3,4> W_pinv = W_.transpose() * (W_ * W_.transpose()).inverse();
  return W_pinv * w;
}

void MecanumKinematics::linearize(double theta,
                                 Eigen::Matrix3d& A,
                                 Eigen::Matrix<double,3,3>& B) const
{
  // 순간선형화 A, B 행렬 계산
  // A = [0, 0, -v*sin(θ);
  //      0, 0,  v*cos(θ);
  //      0, 0,  0]
  // B = [cos(θ), -sin(θ), 0;
  //      sin(θ),  cos(θ), 0;
  //      0,       0,      1]
  
  double c = cos(theta), s = sin(theta);
  
  A.setZero();
  A(0,2) = -s;  // -sin(θ)
  A(1,2) =  c;  //  cos(θ)
  
  B.setZero();
  B(0,0) =  c;  B(0,1) = -s;
  B(1,0) =  s;  B(1,1) =  c;
  B(2,2) = 1.0;
} 