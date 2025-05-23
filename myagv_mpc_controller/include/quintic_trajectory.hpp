#pragma once
#include <vector>
#include <array>

/// 5-차 다항 궤적 샘플 구조
struct Waypoint
{
  double x{}, y{}, theta{};
  double vx{}, vy{}, omega{};
};

/// Quintic trajectory generator
class QuinticTrajectory
{
public:
  /**
   * @param start  초기 상태 (속도 0, 가속 0 가정)
   * @param goal   목표 상태 (속도 0, 가속 0 가정)
   * @param T      총 이동 시간 [s]
   * @param dt     샘플 간격 [s] (default 0.05 s)
   */
  static std::vector<Waypoint> generate(const Waypoint& start,
                                        const Waypoint& goal,
                                        double T,
                                        double dt = 0.05);

private:
  /// 6×6 선형 방정식을 풀어 계수 a0…a5를 채운다
  static void solveCoeffs(const std::array<double,6>& bc,
                          std::array<double,6>& a);
}; 