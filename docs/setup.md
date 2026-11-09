# 외부 PC 준비

## 실행 위치

Nav2·SLAM·SICK 드라이버는 **외부 PC의 Jazzy 환경 또는 개발 컨테이너 안**에서 실행합니다. SSH로 접속한 Husky의 Humble 셸에 Jazzy 설정을 source하지 않습니다.

기존 개발 컨테이너를 사용하는 경우 호스트에서 다음과 같이 진입합니다.

```bash
docker exec -it wsyoon_sol_robot_1.1.2 bash
```

Podman 환경에서는 `docker` 대신 `podman`을 사용합니다. 컨테이너 안에서:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 pkg prefix rwh_beyond_bringup
```

## 새 Jazzy 워크스페이스에 설치

기존 이미지에 설치되어 있다면 이 절차를 반복하지 않습니다. ROS 2 Jazzy, colcon, rosdep이 준비된 외부 PC에서 실행합니다.

```bash
mkdir -p ~/rwh_ws/src
cd ~/rwh_ws/src
git clone https://github.com/ggh-png/rwh_beyond.git
cd ~/rwh_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix rwh_beyond_bringup
ros2 pkg prefix sick_scan_xd
ros2 pkg prefix nav2_route
ros2 pkg prefix opennav_docking
```

센서 launch는 `sick_scan_xd`의 `sick_generic_caller`와 `sick_tim_5xx.launch`를 사용합니다. 해당 패키지가 해결되지 않으면 [SICK 공식 설치 안내](https://github.com/SICKAG/sick_scan_xd)를 따라 Jazzy 의존성을 먼저 설치합니다. 기존 검증 이미지는 Nav2 **1.3.13**을 사용하므로 다른 버전에서는 launch에 사용한 패키지와 플러그인이 있는지 확인합니다.

## DDS 통신 환경 {#dds}

본체와 외부 PC의 **ROS_DOMAIN_ID를 같게** 설정하고, 문제를 줄이려면 설치된 RMW도 맞춥니다. 실제 값은 [본체 확인](connection.md)에서 읽은 값을 사용하며 이 문서는 domain을 임의로 지정하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
# 예: 아래 0은 본체에서 확인한 ROS_DOMAIN_ID로 바꿉니다.
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 daemon stop
ros2 topic list -t
```

RMW를 명시할 경우 본체와 맞는 구현이 외부 PC에도 설치되어 있어야 합니다. `CYCLONEDDS_URI`가 loopback 전용 인터페이스를 지정하거나 discovery server를 별도로 쓰는 환경이면 `SUBNET` 설정만으로 해결되지 않습니다. 기존 DDS 설정과 방화벽, 인터페이스를 함께 확인합니다.

Linux 개발 컨테이너는 로봇 LAN에 접근할 수 있는 네트워크 설정이 필요합니다. 이 프로젝트의 Docker 실행 스크립트는 host network 사용을 기준으로 합니다. 네트워크 모드 변경은 컨테이너 재생성이 필요합니다.

```bash
ros2 run rwh_beyond_bringup inspect_robot
```

이 명령은 그래프의 토픽 타입과 QoS를 확인하며 속도 명령을 발행하지 않습니다. 아직 센서를 시작하지 않았다면 scan/센서 TF가 없을 수 있습니다. 그래프에 보이는 것과 실제 데이터가 도착하는 것은 별개이므로 `ros2 topic echo`로 수신도 확인합니다.
