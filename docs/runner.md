# ros_runner로 반복 운용

`ros_runner`는 [별도 저장소](https://github.com/ggh-png/ros_runner)입니다. 기존 개발 컨테이너의 `ros2_ws`에 설치되어 있으며 `config/runners.yaml`이 실행 명령을 정의합니다.

## runner 시작

**컨테이너 안**에서 ROS 통신 환경을 본체와 맞춘 뒤 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RWH_MAP_FILE=/root/wsyoon_data/map/map.yaml
ros2 launch ros_runner runner.launch.py \
  config_yaml:=/root/ros2_ws/src/ros_runner/config/runners.yaml
```

별도 워크스페이스에 RWH를 빌드했다면 runner 시작 전 `RWH_WORKSPACE`를 그 워크스페이스 루트로 지정합니다. 기본 fallback은 `/root/rwh_beyond`이며 현재 이미지에서는 `/root/ros2_ws/install/setup.bash`도 읽습니다. `logging: 2`는 로그 저장과 터미널 출력을 모두 사용합니다.

## 모드 선택

다른 터미널에서도 같은 ROS 환경을 source한 뒤 필요한 명령 **하나씩** 실행합니다.

```bash
# LiDAR와 센서 TF만 실행
ros2 service call /ros_runner/activate_lidar std_srvs/srv/Empty '{}'

# SLAM + Nav2 + 센서 + RViz
ros2 service call /ros_runner/activate_slam std_srvs/srv/Empty '{}'

# SLAM 실행 중 지도 저장
ros2 service call /ros_runner/activate_map_save std_srvs/srv/Empty '{}'

# 저장한 지도로 AMCL + Nav2 + 센서 + RViz
ros2 service call /ros_runner/activate_navigation std_srvs/srv/Empty '{}'
```

`lidar`, `slam`, `navigation`은 서로를 종료한 뒤 실행합니다. bringup이 센서를 포함하므로 navigation/SLAM의 `requires`에 lidar를 추가하지 않습니다. 전환 시 센서와 RViz도 재시작합니다.

`map_save`는 SLAM을 의존성으로 시작할 수 있으므로, 실제 지도 작성이 완료된 상태에서 호출합니다. 원본은 `/root/wsyoon_data/maps/<timestamp>/`에 저장하고, `RWH_MAP_FILE` 경로에 운용용 YAML/PGM을 복사합니다. 처음 navigation을 실행할 때는 해당 지도가 존재해야 합니다.

```bash
ros2 service call /ros_runner/deactivate_all std_srvs/srv/Empty '{}'
```

이 명령은 같은 runner가 관리하는 Panda 등 **모든 러너**도 종료합니다. 수동으로 실행한 launch는 그 터미널에서 별도로 종료합니다.
