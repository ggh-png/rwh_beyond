# TF와 토픽

이 페이지의 이름은 **현재 RWH 기본 설정**입니다. 본체에서 실제로 발행·구독하는 이름과 타입은 `ros2 topic info --verbose`로 확인합니다.

## 데이터 흐름

```text
Husky 본체 ── odometry + odom→base_link ──▶ 외부 Jazzy PC
SICK LiDAR ── 외부 PC의 sick_scan_xd ──▶ scan (cloud)
외부 PC    ── AMCL 또는 SLAM + Nav2 ──▶ Husky cmd_vel
```

## 주요 토픽

기본 namespace는 `/a200_0724`입니다.

| 토픽 | 메시지 타입 | 역할 / 발행·구독 |
| --- | --- | --- |
| `/a200_0724/platform/odom/filtered` | `nav_msgs/msg/Odometry` | 본체 발행 → Nav2가 참조 |
| `/a200_0724/scan` | `sensor_msgs/msg/LaserScan` | SICK 드라이버 발행 → AMCL/SLAM/costmap/collision monitor |
| `/a200_0724/tf` | `tf2_msgs/msg/TFMessage` | 동적 TF |
| `/a200_0724/tf_static` | `tf2_msgs/msg/TFMessage` | 정적 TF |
| `/a200_0724/map` | `nav_msgs/msg/OccupancyGrid` | map_server 또는 SLAM Toolbox 발행 |
| `/a200_0724/cmd_vel_nav` | `geometry_msgs/msg/Twist` | controller/behavior → velocity smoother |
| `/a200_0724/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | velocity smoother → collision monitor |
| `/a200_0724/cmd_vel` | `geometry_msgs/msg/Twist` | collision monitor 출력 → 본체 구독 기대 |
| `/a200_0724/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | collision monitor가 참조하는 footprint |

일반 경로 추종 시 속도 명령은 `cmd_vel_nav → cmd_vel_smoothed → cmd_vel` 순서입니다. 토픽이 있어도 본체 구독자가 없으면 로봇은 움직이지 않습니다. 실제 본체가 다른 입력 토픽이나 `TwistStamped`를 사용하면 Nav2 출력과 본체 입력을 맞춰야 합니다.

## TF 트리

```text
map
 └── odom             AMCL 또는 SLAM Toolbox (둘 중 하나)
      └── base_link   Husky 본체의 동적 TF
           └── cloud RWH sensor.launch.py의 정적 TF
```

| 변환 | 담당 | 주의점 |
| --- | --- | --- |
| `map → odom` | AMCL / SLAM Toolbox | 동시에 발행하지 않음 |
| `odom → base_link` | Husky 본체 | RWH는 외부 EKF를 실행하지 않음 |
| `base_link → cloud` | RWH 센서 launch | 기존 발행자가 있으면 RWH TF를 끔 |

**토픽 namespace와 frame ID는 다릅니다.** TF 토픽은 `/a200_0724/tf`이지만 frame은 `base_link`, `cloud`입니다. frame ID 앞에 `/a200_0724/`를 임의로 붙이지 않습니다.

### 센서 위치와 회전

`nav2_params.yaml`의 `sensor_config.ros__parameters.scan_static_tf`:

| 값 | 설정 |
| --- | --- |
| parent / child | `base_link` / `cloud` |
| x / y / z (m) | `0.402 / 0.0 / 0.156` |
| roll / pitch / yaw (rad) | `3.141592 / 0.0 / 0.0` |

회전은 **roll 약 180도**입니다. 예전 positional 명령의 yaw/pitch/roll 순서와 혼동하여 yaw에 넣지 않습니다. 현재 launch는 `--roll`, `--pitch`, `--yaw`를 명시합니다. 이 값은 제공된 장착 변환을 옮긴 것으로, 센서를 이동하거나 교체하면 실측하여 수정합니다.

## 확인 명령

```bash
ros2 run tf2_ros tf2_echo base_link cloud --ros-args \
  -r /tf:=/a200_0724/tf -r /tf_static:=/a200_0724/tf_static
ros2 run tf2_ros tf2_echo odom base_link --ros-args \
  -r /tf:=/a200_0724/tf -r /tf_static:=/a200_0724/tf_static
ros2 run tf2_ros tf2_echo map base_link --ros-args \
  -r /tf:=/a200_0724/tf -r /tf_static:=/a200_0724/tf_static
ros2 topic info /a200_0724/cmd_vel --verbose
ros2 topic echo /a200_0724/scan --once --qos-reliability best_effort
```

본체 TF가 `/tf`에만 나오면 현재의 namespaced TF 리스너와 연결되지 않습니다. 본체와 외부 PC의 remapping을 일치시켜야 하며, 움직이는 `odom → base_link`를 임의의 정적 TF로 대체해서는 안 됩니다.
