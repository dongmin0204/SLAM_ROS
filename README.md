# SLAM ROS Package

1. 로봇의 움직임을 제어하고 기록하는 기능을 제공. 
2. MoveXY Action을 통해 로봇을 특정 좌표로 이동시키고, 모든 움직임을 CSV 파일에 기록.

## 주요 기능

1. **로봇 움직임 제어**
   - MoveXY Action을 통한 로봇 이동 명령
   - 실시간 피드백 제공
   - 이동 취소 기능

2. **움직임 기록**
   - 로봇의 모든 움직임을 CSV 파일에 기록
   - 타임스탬프, 위치, 방향 정보 저장
   - 나중에 분석을 위한 데이터 저장

## 필수 요구사항

- ROS 2 Galactic
- CMake 3.8 이상
- C++14 지원 컴파일러

## 의존성 패키지

- rclcpp
- rclcpp_action
- geometry_msgs
- nav_msgs
- sensor_msgs
- tf2
- tf2_ros

## 빌드 방법

1. 워크스페이스에 패키지 복제:
```bash
cd ~/ros2_ws/src
git clone https://github.com/dongmin0204/SLAM_ROS.git
```

2. 패키지 빌드:
```bash
cd ~/ros2_ws
colcon build --packages-select slam_ros
```

3. 워크스페이스 소스:
```bash
source ~/ros2_ws/install/setup.bash
```

## 사용 방법

### 1. 컨트롤러 실행

```bash
ros2 launch slam_ros movement_controller.launch.py
```

### 2. 로봇 이동 명령 보내기

```bash
# 기본 사용법
ros2 action send_goal /move_xy slam_ros/action/MoveXY "{x: 1.0, y: 0.5}"

# 피드백 확인
ros2 action send_goal /move_xy slam_ros/action/MoveXY "{x: 1.0, y: 0.5}" --feedback
```

### 3. 기록된 데이터 확인

로봇의 움직임은 `robot_movements.csv` 파일에 저장됩니다. 파일 형식:
```
timestamp,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w
```

## Action 메시지 구조

### Goal
```yaml
float64 x  # 목표 x 좌표
float64 y  # 목표 y 좌표
```

### Result
```yaml
bool success  # 이동 성공 여부
```

### Feedback
```yaml
float64 current_x  # 현재 x 좌표
float64 current_y  # 현재 y 좌표
```

## 문제 해결

1. 빌드 오류 발생 시:
   - ROS 2 Galactic이 올바르게 설치되어 있는지 확인
   - 모든 의존성 패키지가 설치되어 있는지 확인
   - 워크스페이스를 다시 빌드

2. 실행 오류 발생 시:
   - 워크스페이스가 올바르게 소스되었는지 확인
   - 필요한 토픽이 발행되고 있는지 확인
   - 로그 메시지 확인

MIT License

Copyright (c) 2024 Dongmin Baek

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

