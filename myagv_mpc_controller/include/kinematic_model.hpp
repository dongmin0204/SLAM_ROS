#pragma once
#include <Eigen/Core>

/// 메카넘 kinematics + Jacobian
class MecanumKinematics
{
public:
  /**
   * @param r       휠 반경 (m) – 0.0375 m
   * @param lxly    (Lx+Ly)/2 (m) – 0.135 m
   */
  explicit MecanumKinematics(double r = 0.0375, double lxly = 0.135);

  /// 바디 twist → 4-휠 각속도(rad/s)
  Eigen::Vector4d bodyToWheel(const Eigen::Vector3d& twist) const;

  /// 4-휠 각속도 → 바디 twist (가상)
  Eigen::Vector3d wheelToBody(const Eigen::Vector4d& w) const;

  /// θ 주어진 순간선형화 A, B 행렬(3×3) 구하기
  void linearize(double theta,
                 Eigen::Matrix3d& A,
                 Eigen::Matrix<double,3,3>& B) const;

  /// Jacobian 행렬 반환 (4×3, 내부 캐시)
  const Eigen::Matrix<double,4,3>& W() const { return W_; }

private:
  double r_, lxly_;
  Eigen::Matrix<double,4,3> W_;
}; 