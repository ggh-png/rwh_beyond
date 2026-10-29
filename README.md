# rwh_beyond

ROS 2 Jazzy / Nav2 기반 Husky 외부 PC navigation과 커스텀 서비스 확장을 위한
워크스페이스입니다. `opennav_coverage` Jazzy의 **서버 / 인터페이스 / bringup**
분리를 참고했고, coverage 기능과 Fields2Cover 의존성은 포함하지 않습니다.

## 저장소 구조

참고 프로젝트처럼 ROS 패키지를 저장소 루트에 둡니다. 저장소 루트에서 바로
`colcon build`할 수 있고, 다른 workspace의 `src/` 아래에 clone해도 됩니다.

```text
rwh_beyond/
├── rwh_beyond/                   # ament_cmake, C++ lifecycle/component server
│   ├── include/rwh_beyond/beyond_server.hpp
│   ├── src/beyond_server.cpp
│   ├── src/main.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── rwh_beyond_msgs/              # rosidl, 커스텀 msg/srv/action 정의
│   └── srv/GetStatus.srv
├── rwh_beyond_bringup/           # launch 및 전체 파라미터, Python 실행 유틸리티
│   ├── launch/
│   │   ├── navigation.launch.py
│   │   ├── server.launch.py
│   │   └── scan_tf.launch.py
│   ├── params/
│   │   ├── nav2_params.yaml      # 실제 사용, 전체 편집 가능
│   │   ├── rwh_beyond.yaml       # 커스텀 서버 파라미터
│   │   ├── sensor_tf.yaml        # base_link → cloud
│   │   └── upstream/            # Nav2 기본 YAML 원본 전체
│   └── rwh_beyond_bringup/       # namespace 처리, TF, 읽기 전용 토픽 검사
├── tests/
└── docs/upstream.md              # 구조 매핑과 원본 버전/해시
```

`rwh_beyond::BeyondServer`는 `nav2_util::LifecycleNode`이며 shared library,
standalone executable, ROS component로 빌드됩니다. 현재 서비스는 읽기 전용
`GetStatus` 하나입니다. 로봇 이동/coverage 동작을 구현하지 않습니다. BT나 navigator
plugin은 실제 기능이 필요해질 때 별도 패키지로 추가합니다.

## 전체 Nav2 파라미터

`rwh_beyond_bringup/params/nav2_params.yaml`을 직접 수정합니다. 실행마다 설치된
Nav2 기본 파일을 읽어 일부 값만 만드는 이전 방식은 사용하지 않습니다.
원본 전체도 `params/upstream/`에 보관하고 버전/해시를 기록했습니다.

AMCL, BT navigator, controller, local/global costmap, map saver, planner, smoother,
behavior, waypoint, route, velocity smoother, collision monitor, docking, loopback
simulator 등 원본 기본 YAML의 15개 섹션이 포함됩니다. simulator 섹션이 있다고
실물 launch에서 simulator가 실행되는 것은 아닙니다.

launch는 다음 값만 처리합니다.

- `<robot_namespace>`: `/a200_0724` 등 실제 namespace로 치환.
- `use_sim_time`: 모든 ROS 파라미터 블록에 적용.
- `cmd_vel_type`: 명시한 경우에만 전체 속도 체인의 stamped 여부 변경.

그 외 plugin, 속도, costmap, footprint 등은 파일 값 그대로 사용합니다.
별도 로봇 설정 파일에서 튜닝값을 다시 덮어쓰지 않습니다.

## 기본 인터페이스

| 항목 | 기본값 |
| --- | --- |
| namespace | `a200_0724` |
| base frame | `base_link` |
| scan | `/a200_0724/scan`, frame_id=`cloud` |
| odometry | `/a200_0724/platform/odom/filtered` |
| cmd_vel | `/a200_0724/cmd_vel`, `Twist` |
| TF | `/a200_0724/tf`, `/a200_0724/tf_static` |

명령 타입은 Humble Clearpath 문서 기준의 가정이며 실제 본체에서 확인해야 합니다.
Jazzy 본체 등에서 stamped 명령을 요구하면 YAML의 `enable_stamped_cmd_vel` 값을
일관되게 바꾸거나 launch에 `cmd_vel_type:=twist_stamped`를 지정합니다.

footprint는 예제입니다. 실제 차체/상부 장착물 실측값으로 수정해야 합니다.
기본 속도 0.25m/s, 0.5rad/s도 초기 예제값이며 실물 주행 튜닝은 별도입니다.

## 빌드

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup python3-colcon-common-extensions python3-yaml
source /opt/ros/jazzy/setup.bash
cd ~/rwh_beyond
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

기본 의존성은 Nav2, tf2, lifecycle/component, rosidl입니다. Clearpath navigation
패키지나 robot.yaml 파서에 의존하지 않습니다. 본체의 드라이버, EKF, sensor,
robot_state_publisher, twist mux, 비상정지 시스템은 본체에서 계속 동작합니다.

## Navigation 실행

```bash
ros2 launch rwh_beyond_bringup navigation.launch.py \
  map:=/root/wsyoon_data/real_map_0505.yaml \
  namespace:=a200_0724 \
  publish_scan_tf:=true autostart:=true
```

공식 `nav2_bringup/bringup_launch.py`를 include하며, RWH 서버는 별도 lifecycle
manager로 관리합니다. `start_server:=false`이면 Nav2만 실행합니다.
기본 `autostart=false`는 초기 설정 확인용입니다.

실제로 편집 중인 파라미터 파일을 명시할 수도 있습니다.

```bash
ros2 launch rwh_beyond_bringup navigation.launch.py \
  map:=/root/wsyoon_data/real_map_0505.yaml \
  params_file:=/root/rwh_beyond/rwh_beyond_bringup/params/nav2_params.yaml \
  autostart:=true
```

share 경로의 파라미터 파일은 빌드 시 설치됩니다. YAML 수정 후 재빌드하거나
`params_file`로 원본 파일을 직접 지정합니다. 옵션 `server_params_file`과
`sensor_tf_file`로 서버/TF 설정도 따로 지정할 수 있습니다.

## 커스텀 서버만 실행

```bash
ros2 launch rwh_beyond_bringup server.launch.py namespace:=a200_0724 autostart:=true
ros2 service call /a200_0724/beyond_server/get_status rwh_beyond_msgs/srv/GetStatus '{}'
```

`GetStatus`는 unconfigured/inactive 상태에서도 조회 가능합니다. 새 업무 서비스는
`rwh_beyond_msgs/srv/`에 정의하고 해당 CMake에 등록한 후 `rwh_beyond`에 callback을
구현합니다. 실제 작업을 수행하는 서비스는 lifecycle active 상태를 확인하도록
구현합니다. 취소/진행 피드백이 필요한 긴 작업은 action을 추가하는 편이 적합합니다.

컴포넌트 이름은 `rwh_beyond::BeyondServer`, 실행 파일은
`ros2 run rwh_beyond rwh_beyond_server`입니다.

## cloud TF

사용자가 제공한 legacy 위치 인자 `0.402 0 0.156 0 0 3.141592 base_link cloud`는
`x y z yaw pitch roll` 순서입니다. `params/sensor_tf.yaml`에는 다음을 보존했습니다.

- parent `base_link`, child `cloud`
- translation `[0.402, 0, 0.156]` m
- roll/pitch/yaw `[3.141592, 0, 0]` rad

중복 publisher를 피하도록 기본값은 꺼져 있습니다. 위 navigation의
`publish_scan_tf:=true` 또는 아래 standalone 방식 중 하나를 사용합니다.

```bash
ros2 launch rwh_beyond_bringup scan_tf.launch.py namespace:=a200_0724
```

scan의 frame_id를 문자열만 바꾸지 않으며 실제 TF를 연결합니다. TF는 Nav2와
같은 namespace로 발행합니다. 본체 TF가 전역 `/tf`라면 namespace를 비우고
파라미터의 하드웨어 토픽은 필요한 절대 경로로 지정합니다.

## 외부 PC와 Humble 본체

Humble↔Jazzy DDS 통신은 이 저장소가 호환성을 보장하지 않습니다. 먼저 본체 버전,
`cmd_vel` 타입, scan/odom 실제 데이터, TF, QoS와 시간 동기화를 확인해야 합니다.
Nav2 action client/server와 RViz는 같은 Jazzy 환경에 두는 구성을 권장합니다.
본체가 Humble인 경우 외부 PC에서 Humble Docker로 navigation 배포판을 맞추는
방법도 고려할 수 있습니다.

기존 wsyoon 이미지의 `.ros2rc`는 discovery를 LOCALHOST로 설정합니다. 외부 로봇과
연결할 때는 컨테이너에 들어가 환경을 source한 후, 본체 domain/RMW에 맞춥니다.

```bash
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=0  # 예제: 실제 본체 값으로 변경
ros2 daemon stop
ros2 run rwh_beyond_bringup inspect_robot
```

이 검사는 graph/type/QoS 조회만 하며 속도를 발행하지 않습니다. 데이터 수신을
입증하는 검사는 별도로 필요합니다. 컨테이너 host networking, 방화벽, multicast,
DDS profile과 양 PC의 NTP/chrony 설정도 확인합니다. `map -> odom`은 AMCL 등 한
노드만 제공하도록 하고, 본체의 기존 navigation과 속도/TF 발행이 중복되지 않게 합니다.

## 검증

Jazzy / Nav2 1.3.13 격리 컨테이너(`--network none`)에서 확인했습니다.
실제 로봇과 센서 드라이버는 시작하지 않았습니다.

```bash
python3 tests/test_parameters.py
python3 tests/verify_server.py
python3 tests/verify_scan_tf.py
python3 tests/verify_bringup.py --log-dir /tmp/rwh-twist --cloud-frame
python3 tests/verify_bringup.py --log-dir /tmp/rwh-stamped --cloud-frame --cmd-vel-type twist_stamped
```

전체 설정 유지/튜닝 보존 7건, custom service 및 lifecycle 전환, component load/unload,
기존 cloud TF와의 수치 일치, Nav2 12개 + RWH 서버 1개 활성화를 확인합니다.
실제 Humble 본체 통신과 실제 주행은 미검증입니다.

[구조 참고 및 전체 파라미터 출처](docs/upstream.md)
