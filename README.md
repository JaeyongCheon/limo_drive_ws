# LIMO Pro Autonomous Navigation Workspace

LIMO Pro 로봇을 위한 ROS2 자율주행 네비게이션 워크스페이스입니다. SLAM 기반 매핑, Nav2 네비게이션, 지형 분석, YOLO 객체 감지 통합 기능을 포함합니다.

## 시스템 요구사항

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- LIMO Pro 로봇 하드웨어
  - 2D LiDAR 센서
  - Orbbec Dabai Depth Camera
  - IMU

## 패키지 구성

### remo_navigation
LIMO Pro의 자율주행 네비게이션 메인 패키지
- **자율주행 런치**: `autonomous_navigation_limo.launch.py`
- **Nav2 설정**: `nav2_params_slam.yaml`
- **YOLO 통합**: `yolo_location_saver.py`
- **지형 분석**: Terrain analysis integration

### grid_map
ANYbotics의 grid_map 패키지 (포함됨)
- 지형 데이터를 costmap으로 변환
- 다층 그리드 맵 생성 및 관리

### terrain_analysis
지형 분석 패키지
- 경사도, 거칠기, 고도 분석
- Costmap 생성 및 발행

## 빌드 방법

```bash
cd ~/drive_ws
colcon build --symlink-install
source install/setup.bash
```

특정 패키지만 빌드:
```bash
colcon build --packages-select remo_navigation
```

## 실행 방법

### 1. 기본 자율주행 (SLAM + Nav2)

**터미널 1: 자율주행 시스템 실행**
```bash
cd ~/drive_ws
source install/setup.bash
ros2 launch remo_navigation autonomous_navigation_limo.launch.py
```

이 명령어는 다음을 실행합니다:
- TF 변환 (laser, camera)
- Terrain Analysis 노드
- Grid Map to Costmap 변환기
- SLAM Toolbox (매핑 모드)
- Nav2 네비게이션 스택
- RViz2 시각화

**터미널 2: 네비게이션 목표 설정**
- RViz2에서 "2D Goal Pose" 버튼 사용
- 또는 CLI로 목표 전송:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'odom'
pose:
  position: {x: 2.0, y: 1.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

### 2. YOLO 객체 감지와 위치 저장

YOLO로 감지된 객체(사람, 자동차, 병)의 위치를 자동으로 저장합니다.

**터미널 1: Orbbec 카메라 실행**
```bash
ros2 launch orbbec_camera dabai.launch.py
```

**터미널 2: YOLO 감지 실행**
```bash
cd ~/Downloads/py_bt_ros_spy
source install/setup.bash
ros2 launch yolo_bringup yolo.launch.py
```

**터미널 3: YOLO 위치 저장 노드 실행**
```bash
source ~/Downloads/py_bt_ros_spy/install/setup.bash
source ~/drive_ws/install/setup.bash
ros2 run remo_navigation yolo_location_saver.py
```

**터미널 4 (선택사항): 자율주행 시스템**
```bash
cd ~/drive_ws
source install/setup.bash
ros2 launch remo_navigation autonomous_navigation_limo.launch.py
```

감지된 객체의 위치는 `~/yolo_detections.json`에 저장되며, `/yolo_waypoint` 토픽으로 발행됩니다.

### 3. 저장된 맵 사용 (Localization)

맵을 저장한 후 로컬라이제이션 모드로 실행:

**맵 저장:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

**로컬라이제이션 모드 실행:**
```bash
ros2 launch remo_navigation autonomous_navigation_limo.launch.py \
  use_sim_time:=false \
  map:=~/maps/my_map.yaml \
  slam:=false
```

## 주요 토픽

### 입력 토픽
- `/scan` - 2D LiDAR 데이터 (sensor_msgs/LaserScan)
- `/odom` - 오도메트리 (nav_msgs/Odometry)
- `/camera/depth/points` - Depth 포인트클라우드 (sensor_msgs/PointCloud2)
- `/yolo/detections` - YOLO 감지 결과 (yolo_msgs/DetectionArray)

### 출력 토픽
- `/map` - SLAM 생성 맵 (nav_msgs/OccupancyGrid)
- `/local_costmap/costmap` - 로컬 코스트맵
- `/global_costmap/costmap` - 글로벌 코스트맵
- `/plan` - 계획된 경로 (nav_msgs/Path)
- `/yolo_waypoint` - YOLO 감지 위치 (geometry_msgs/PoseStamped)

## 설정 파일

### Nav2 파라미터 (`config/nav2_params_slam.yaml`)
주요 설정:
- **Robot Footprint**: 반경 0.17m (local), 0.25m (global)
- **Inflation Radius**: 0.6m (local), 1.0m (global)
- **Transform Tolerance**: 2.0초
- **Max Velocity**: 선형 0.26 m/s, 각속도 1.0 rad/s
- **Planner Tolerance**: 1.5m
- **DWB Controller**: ObstacleFootprint critic 사용

### YOLO 감지 설정 (`scripts/yolo_location_saver.py`)
- **감지 클래스**: person, car, bottle
- **최소 신뢰도**: 0.5
- **저장 위치**: `~/yolo_detections.json`

## 문제 해결

### TF Transform 에러
```bash
# Transform tolerance 증가 (이미 설정됨)
# nav2_params_slam.yaml에서 transform_tolerance: 2.0
```

### Costmap이 업데이트되지 않음
```bash
# 센서 토픽 확인
ros2 topic hz /scan
ros2 topic hz /camera/depth/points

# Costmap 토픽 확인
ros2 topic echo /local_costmap/costmap
```

### YOLO 모듈을 찾을 수 없음
```bash
# 두 워크스페이스 모두 source
source ~/Downloads/py_bt_ros_spy/install/setup.bash
source ~/drive_ws/install/setup.bash
```

### Navigation이 실패함
```bash
# Recovery behavior 파라미터 확인
# 장애물 회피 설정 확인 (inflation_radius)
# 로봇 주변 공간 확보
```

## 개발 정보

### 빌드 시스템
- remo_navigation: CMake 기반 패키지
- Python 스크립트는 `scripts/` 디렉토리에 위치

### 주요 의존성
- nav2_bringup
- slam_toolbox
- grid_map_ros
- dwb_core
- yolo_msgs (별도 설치 필요)

## 라이선스

이 워크스페이스는 여러 오픈소스 패키지를 포함합니다. 각 패키지의 라이선스를 참조하세요.

## 참고 자료

- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Grid Map](https://github.com/ANYbotics/grid_map)
- [LIMO Pro](https://www.agilex.ai/)

## 기여

버그 리포트 및 기여는 GitHub Issues를 통해 환영합니다.
