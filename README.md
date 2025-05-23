# SLAM_ROS 프로젝트

## 개요
이 프로젝트는 ROS 2 Galactic을 사용하여 SLAM(Simultaneous Localization and Mapping)과 MPC(Model Predictive Control)를 구현한 자율주행 시스템입니다. 프로젝트는 두 개의 주요 패키지로 구성되어 있습니다:

1. `slam_ros`: 움직임 기록 기능이 포함된 핵심 SLAM 구현
2. `myagv_mpc_controller`: MPC 기반 궤적 추적 제어기

## 사전 요구사항

### 시스템 요구사항
- macOS (ARM64/Apple Silicon)
- ROS 2 Galactic
- CMake 3.8 이상
- C++14 호환 컴파일러

### 의존성 패키지
- Eigen3
- OMPL
- PCL (Point Cloud Library)
- OpenCV
- GTSAM
- Ceres Solver

## 설치 방법

### 1. ROS 2 Galactic 설치
```bash
# Homebrew를 통한 ROS 2 Galactic 설치
brew install ros-galactic

# ROS 2 환경 설정
source /opt/homebrew/opt/ros/galactic/setup.bash
```

### 2. 프로젝트 설정
```bash
# 저장소 클론
git clone https://github.com/dongmin0204/SLAM_ROS.git
cd SLAM_ROS

# 워크스페이스 빌드
colcon build --symlink-install
```

### 3. VSCode 설정
1. 다음 VSCode 확장 프로그램 설치:
   - C/C++
   - CMake Tools
   - Python
   - ROS

2. C/C++ include 경로 설정:
   - 프로젝트에 포함된 `.vscode/c_cpp_properties.json` 파일에 올바른 include 경로가 설정되어 있습니다
   - 설치 후 VSCode를 재시작하세요

## 프로젝트 구조

### slam_ros 패키지
```
slam_ros/
├── include/
│   ├── slam_ros/
│   │   ├── movement_controller.hpp
│   │   ├── movement_recorder.hpp
│   │   └── slam_node.hpp
├── src/
│   ├── movement_controller.cpp
│   ├── movement_recorder.cpp
│   └── slam_node.cpp
├── launch/
│   └── slam.launch.py
├── config/
│   └── slam_params.yaml
├── CMakeLists.txt
└── package.xml
```

### myagv_mpc_controller 패키지
```
myagv_mpc_controller/
├── include/
│   └── myagv_mpc_controller/
│       ├── quintic_trajectory.hpp
│       ├── kinematic_model.hpp
│       └── mpc_solver.hpp
├── src/
│   ├── quintic_trajectory.cpp
│   ├── kinematic_model.cpp
│   └── mpc_solver.cpp
├── launch/
│   └── mpc_controller.launch.py
├── config/
│   └── mpc_params.yaml
├── CMakeLists.txt
└── package.xml
```

## 사용 방법

### SLAM 실행
```bash
# 워크스페이스 소스
source install/setup.bash

# SLAM 노드 실행
ros2 launch slam_ros slam.launch.py
```

### MPC 컨트롤러 실행
```bash
# MPC 컨트롤러 실행
ros2 launch myagv_mpc_controller mpc_controller.launch.py
```

## 개발 가이드

### 빌드
```bash
# 특정 패키지 빌드
colcon build --packages-select slam_ros
colcon build --packages-select myagv_mpc_controller

# 모든 패키지 빌드
colcon build
```

### 테스트
```bash
# 테스트 실행
colcon test
colcon test-result --verbose
```

## 문제 해결

### 일반적인 문제

1. Include 경로 오류
   - ROS 2 환경이 제대로 설정되어 있는지 확인
   - VSCode C/C++ 설정 확인
   - ROS 2 설치 경로 확인

2. 빌드 오류
   - 모든 의존성 패키지가 설치되어 있는지 확인
   - CMake 설정 확인
   - 컴파일러 호환성 확인

3. 실행 오류
   - ROS 2 노드 연결 확인
   - 파라미터 설정 확인
   - 시스템 리소스 사용량 확인

## 기여 방법
1. 저장소 포크
2. 기능 브랜치 생성
3. 변경사항 커밋
4. 브랜치 푸시
5. Pull Request 생성

## 라이선스
이 프로젝트는 MIT 라이선스 하에 배포됩니다. 자세한 내용은 LICENSE 파일을 참조하세요.

## 감사의 말
- ROS 2 커뮤니티
- Open Source Robotics Foundation
- 기여자 및 유지보수자들

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

