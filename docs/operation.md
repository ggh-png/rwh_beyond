# SLAM과 Nav2 실행

아래 명령은 **외부 Jazzy PC/컨테이너**에서 ROS와 워크스페이스 환경을 source한 후 실행합니다. 본체 Clearpath 서비스는 실행 중이어야 합니다. 수동 launch와 runner 운용을 동시에 시작하지 않습니다.

## 센서만 점검

```bash
ros2 launch rwh_beyond_bringup sensor.launch.py \
  namespace:=a200_0724 sensor_hostname:=172.16.0.161
```

다른 터미널에서:

```bash
ros2 topic echo /a200_0724/scan --once --qos-reliability best_effort
ros2 topic hz /a200_0724/scan
```

`header.frame_id`가 `cloud`인지 확인합니다. 통합 bringup을 시작하기 전 이 센서 launch를 `Ctrl+C`로 종료합니다. 센서를 계속 따로 실행하려면 bringup에 `start_sensor:=false`를 줍니다.

## SLAM으로 지도 만들기 {#slam}

```bash
ros2 launch rwh_beyond_bringup bringup.launch.py \
  namespace:=a200_0724 slam:=true use_sim_time:=false
```

이 명령은 센서, 정적 TF, SLAM Toolbox, Nav2, RViz를 함께 실행합니다. 별도로 AMCL을 켜지 않습니다. 로봇을 현장 운용 절차에 따라 천천히 이동시키며 RViz의 지도와 scan이 일치하는지 확인합니다.

지도가 완성되면 다른 터미널에서 저장합니다.

```bash
mkdir -p ~/wsyoon_data/map
ros2 run nav2_map_server map_saver_cli \
  -t /a200_0724/map -f ~/wsyoon_data/map/map
```

`map.yaml`과 `map.pgm`이 생성됩니다. 지도 YAML의 `image`가 실제 PGM 파일을 가리켜야 합니다. 컨테이너에서는 영속 마운트된 `/root/wsyoon_data`에 저장합니다.

## 저장한 지도로 주행 {#navigation}

SLAM launch를 종료한 다음 실행합니다.

```bash
ros2 launch rwh_beyond_bringup bringup.launch.py \
  namespace:=a200_0724 slam:=false use_sim_time:=false \
  map_yaml_file:="$HOME/wsyoon_data/map/map.yaml"
```

1. RViz Fixed Frame을 `map`으로 둡니다.
2. **2D Pose Estimate**로 지도상의 현재 위치와 방향을 지정합니다.
3. scan과 지도 벽이 맞는지, 로봇 footprint가 실제 크기와 맞는지 확인합니다.
4. Nav2가 활성화된 뒤 **Nav2 Goal**로 가까운 목표를 지정합니다.

```bash
ros2 lifecycle get /a200_0724/amcl
ros2 lifecycle get /a200_0724/controller_server
ros2 lifecycle get /a200_0724/bt_navigator
```

기본 지도는 패키지의 `real_map_0505.yaml`입니다. 현재 장소와 다른 지도라면 반드시 `map_yaml_file`을 지정합니다. 지정한 지도나 이미지가 없으면 bringup은 시작 단계에서 실패합니다.

## 자주 쓰는 옵션

| 옵션 | 용도 |
| --- | --- |
| `params_file:=/절대경로/nav2_params.yaml` | 사용자 전체 파라미터 파일 |
| `use_rviz:=false` | GUI 없는 PC에서 실행 |
| `start_sensor:=false` | 드라이버와 센서 TF를 이미 다른 곳에서 발행 중 |
| `start_driver:=false` | 센서 TF만 발행, 드라이버 접속은 생략 |
| `use_tf_publisher:=false` | 기존 TF가 있어 RWH 센서 TF만 비활성화 |
| `sensor_hostname:=172.16.0.161` | YAML의 센서 IP 덮어쓰기 |
| `use_sim_time:=true` | 시뮬레이션 시간 사용, 실제 센서 launch 생략 |
| `use_composition:=False` | 기본 실행 방식 유지 |

`start_sensor:=false`는 드라이버와 센서 TF 모두를 생략합니다. 본체에서 scan만 발행하고 `base_link → cloud`가 없다면 `start_driver:=false`로 RWH의 TF는 유지합니다. 같은 센서의 드라이버 또는 동일 TF를 두 군데에서 실행하지 않습니다.

실제 로봇에서는 `use_sim_time:=false`를 사용합니다. 종료는 launch 터미널의 `Ctrl+C`이며, 로봇의 정지가 확인된 뒤 장비 전원을 관리합니다.
