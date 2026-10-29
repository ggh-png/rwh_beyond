# rwh_beyond

ROS 2 Jazzy 기반 Husky 외부 PC navigation과 커스텀 서비스 확장을 위한 workspace입니다.
`opennav_coverage`의 서버 / 인터페이스 / bringup 분리를 참고했으며 coverage는 포함하지 않습니다.

```text
rwh_beyond/                         # C++ lifecycle/component 서버
rwh_beyond_msgs/                    # 커스텀 서비스 GetStatus.srv
rwh_beyond_bringup/
  launch/
    bringup.launch.py              # navigation + localization/SLAM + sensor + RViz
    navigation.launch.py
    localization.launch.py
    slam.launch.py
    sensor.launch.py
    rviz.launch.py
  rviz/navigation.rviz             # RViz 화면 설정
  params/nav2_params.yaml           # 파라미터는 이 파일 하나만 사용
  maps/real_map_0505.yaml            # 지도 메타데이터
  maps/real_map_0505.pgm             # 지도 이미지
```

## 빌드와 실행

```bash
source /opt/ros/jazzy/setup.bash
cd ~/rwh_beyond
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch rwh_beyond_bringup bringup.launch.py
```

기본값은 namespace `a200_0724`, 실제 시간, AMCL, SICK 센서 및 RViz 활성화입니다.
SLAM은 `slam:=true`, RViz 비활성화는 `use_rviz:=false`로 선택합니다.
`use_sim_time:=true`이면 실제 센서 launch는 시작하지 않습니다. 시뮬레이터의 scan/TF가 필요합니다.

```bash
ros2 launch rwh_beyond_bringup bringup.launch.py \
  map_yaml_file:=/path/to/map.yaml \
  params_file:=/path/to/nav2_params.yaml \
  sensor_hostname:=172.16.0.161 \
  use_rviz:=false
```

`map:=...`도 지원하며 `map_yaml_file`을 함께 지정하면 후자가 우선합니다.
기본 지도는 기존 `ros/real_map_0505.*`를 패키지에 복사한 것입니다.
지도 YAML은 지도 메타데이터이므로 `params/`가 아닌 `maps/`에 둡니다.

센서를 다른 runner에서 이미 실행 중이면 `start_sensor:=false`를 지정합니다.
하드웨어 접속 없이 TF만 실행하려면 `start_driver:=false`를 사용합니다.
기본 실행 방식은 일반 프로세스(`use_composition:=False`)입니다. 컴포넌트 모드는
앞선 테스트에서 활성화됐으나 SIGINT 종료 중 crash가 관찰돼 기본으로 사용하지 않습니다.

## 단일 파라미터 파일

모든 튜닝은 `rwh_beyond_bringup/params/nav2_params.yaml`에서 합니다.
Nav2 Jazzy 1.3.13 기본 YAML의 15개 섹션을 유지했고 다음 설정을 통합했습니다.

- `sensor_config.ros__parameters`: SICK hostname, min/max angle, scan frame, 정적 TF
- `slam_toolbox.ros__parameters`: SLAM 프레임, scan 토픽, 해상도 등
- `beyond_server.ros__parameters`: 커스텀 서버 설정

별도의 센서/서버 YAML과 upstream 파라미터 사본은 제거했습니다.
외부 파일을 지정할 때도 위 섹션을 포함한 전체 파일을 전달합니다.
Bringup은 `<robot_namespace>`와 시간 설정을 처리하며 costmap/controller 튜닝은 보존합니다.
센서 TF 활성화 기본값은 `sensor_config.ros__parameters.scan_static_tf.enabled`를 따릅니다.
`use_namespace:=false`이면 센서/TF/Nav2 및 토픽 치환을 전역 namespace로 맞춥니다.

단독 센서 실행도 같은 파일을 사용합니다.

```bash
ros2 launch rwh_beyond_bringup sensor.launch.py
ros2 launch rwh_beyond_bringup sensor.launch.py params_file:=/path/to/nav2_params.yaml
```

`hostname`/`sensor_hostname`, `min_ang`, `max_ang`을 명시하면 YAML 값을 덮어씁니다.
`use_tf_publisher:=false` 또는 기존 `publish_scan_tf:=false`로 센서 TF를 끌 수 있습니다.
SICK는 `sick_scan_xd/sick_generic_caller`와 vendor XML을 사용하는 일반 노드입니다.
Ouster/GPS/DLIO 및 해당 프로젝트의 전용 서버는 포함하지 않습니다.

## 본체 인터페이스

| 항목 | 기본값 |
| --- | --- |
| scan | `/a200_0724/scan`, frame `cloud` |
| odometry | `/a200_0724/platform/odom/filtered` |
| base frame | `base_link` |
| cmd_vel | `/a200_0724/cmd_vel`, `geometry_msgs/msg/Twist` |
| TF | `/a200_0724/tf`, `/a200_0724/tf_static` |

센서 TF는 `base_link -> cloud`, 위치 `[0.402, 0, 0.156]` m,
roll/pitch/yaw `[3.141592, 0, 0]` rad입니다. SICK 내부 TF는 `tf_publish_rate:=0`으로 끕니다.
본체는 `odom -> base_link` TF와 odometry를 제공해야 합니다. 외부 EKF는 실행하지 않습니다.
`map -> odom`은 AMCL 또는 SLAM 한쪽에서만 발행해야 합니다.

footprint와 속도 제한은 초기 예제값이며 실제 로봇에 맞는 튜닝이 필요합니다.
Humble 본체와 Jazzy PC의 실제 DDS/QoS/TF 통신 및 주행은 아직 검증하지 않았습니다.
기존 이미지의 localhost discovery 설정을 해제하고 본체의 domain/RMW에 맞춥니다.

```bash
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 run rwh_beyond_bringup inspect_robot
```

## 검증

하드웨어와 연결하지 않는 격리 환경에서 실행합니다.

```bash
python3 tests/test_parameters.py
python3 tests/verify_sensor_config.py
python3 tests/verify_scan_tf.py
python3 tests/verify_slam.py --log-dir /tmp/rwh-slam
python3 tests/verify_bringup_entrypoint.py --log-dir /tmp/rwh-check --composition false
```

[구조와 파라미터 출처](docs/upstream.md)를 참고하세요.

## RViz 설정

`rwh_beyond_bringup/rviz/navigation.rviz`가 기본 화면 설정입니다. 첨부 설정을 기반으로
지도, LaserScan, TF, global/local costmap, 경로와 Nav2 패널을 포함합니다.
TF 표시 목록에 `map`, `cloud`를 추가했습니다. 화면의 fixed frame은 `map`입니다.
설정의 `<robot_namespace>`는 launch에서 치환하며 TF frame ID에는 namespace를 붙이지 않습니다.

```bash
ros2 launch rwh_beyond_bringup rviz.launch.py
ros2 launch rwh_beyond_bringup rviz.launch.py use_namespace:=false
ros2 launch rwh_beyond_bringup rviz.launch.py rviz_config:=/path/to/custom.rviz
```

Bringup에서도 `use_rviz:=true`일 때 같은 파일을 사용합니다.

## ros_runner 연결

navigation과 SLAM은 통합 bringup으로 센서와 RViz까지 함께 실행합니다.
`lidar`는 센서 단독 점검용입니다. 세 runner는 서로를 종료한 뒤 실행하여 센서/TF
중복을 방지합니다. 모드 전환 시 센서와 RViz도 재시작합니다.

`map_save`는 timestamp 디렉터리에 저장한 지도를 `RWH_MAP_FILE` 경로로 복사합니다.
복사한 YAML의 image 필드도 대상 PGM 이름에 맞춥니다.
기본 지도는 `/root/wsyoon_data/map/map.yaml`, 별도 workspace는
`RWH_WORKSPACE`(기본 `/root/rwh_beyond`)입니다. runner 시작 전에 환경변수를 지정합니다.
외부 EKF는 사용하지 않으며 본체 odometry와 TF가 필요합니다.

```bash
ros2 service call /ros_runner/activate_lidar std_srvs/srv/Empty '{}'
ros2 service call /ros_runner/activate_navigation std_srvs/srv/Empty '{}'
ros2 service call /ros_runner/activate_slam std_srvs/srv/Empty '{}'
ros2 service call /ros_runner/activate_map_save std_srvs/srv/Empty '{}'
ros2 service call /ros_runner/deactivate_all std_srvs/srv/Empty '{}'
```

단독 navigation/localization launch도 namespace와 토픽 placeholder를 처리합니다.
컴포넌트 모드에서 직접 실행한다면 같은 namespace의 외부 container가 필요합니다.

격리 검증에서는 실제 runner 서비스를 호출하되 테스트 사본에서만 SICK 드라이버와
RViz를 끄고 합성 scan/TF를 발행합니다. 실제 센서·GUI·로봇 동작은 시작하지 않습니다.

```bash
python3 tests/verify_server.py
python3 tests/verify_runner_modes.py --runner-config /path/to/ros_runner/config/runners.yaml \
  --workspace /path/to/combined_workspace --log-dir /tmp/rwh-runner-check
```
