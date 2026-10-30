# 파라미터 설정

운용 설정은 `rwh_beyond_bringup/params/nav2_params.yaml` **하나**에 모여 있습니다. 별도 센서 YAML은 없습니다. 지도 YAML은 파라미터와 다른 파일이며 `map_yaml_file` 인자로 전달합니다.

## 수정할 위치

| 목적 | YAML 위치 |
| --- | --- |
| LiDAR IP / scan 각도 | `sensor_config.ros__parameters.hostname`, `min_ang`, `max_ang` |
| scan frame / 장착 TF | `sensor_config.ros__parameters.scan_frame`, `scan_static_tf` |
| AMCL / 위치 추정 | `amcl.ros__parameters` |
| 주행 속도·가속도 | `controller_server.ros__parameters.FollowPath`, `velocity_smoother.ros__parameters` |
| 회전 복구 동작 | `behavior_server.ros__parameters` |
| 로봇 크기 | local/global costmap의 `footprint`, `footprint_padding` |
| 장애물 감지 거리 | costmap의 scan observation 및 `collision_monitor` |
| SLAM 해상도 | `slam_toolbox.ros__parameters.resolution` |

`<robot_namespace>`는 launch가 `/a200_0724`로 치환합니다. namespace를 바꾸려면 우선 launch의 `namespace` 인자를 바꿉니다. 프레임 이름은 자동으로 prefix되지 않습니다.

## 현재 초기 설정

- MPPI: 전진·후진 최대 절댓값 `0.25 m/s`, 회전 최대 `0.5 rad/s`.
- footprint: `[[0.55, 0.4], [0.55, -0.4], [-0.55, -0.4], [-0.55, 0.4]]`, padding `0.03 m`.
- LiDAR 각도: 약 `-π/2`에서 `+π/2`.
- SLAM 해상도: `0.025 m/cell`.
- `enable_stamped_cmd_vel: false`: 현재 출력 타입은 `Twist`.

footprint는 초기 예제 외곽선입니다. 장착물까지 포함한 실제 로봇 외곽을 측정해 local/global costmap 양쪽에 적용합니다. 속도를 변경할 때는 MPPI, velocity smoother, behavior 설정을 함께 확인합니다.

## 수정 후 적용

설치 공간의 파일을 직접 고치기보다 Git 소스 파일을 수정합니다. 빌드 방식에 따라 재빌드가 필요할 수 있으며, 확실하게 적용하려면 전체 YAML 경로를 직접 전달합니다.

```bash
ros2 launch rwh_beyond_bringup bringup.launch.py \
  params_file:=/절대경로/nav2_params.yaml \
  map_yaml_file:=/절대경로/map.yaml
```

launch 시작 시 읽으므로 파일 수정 후에는 해당 launch를 재시작합니다. 파일은 Nav2뿐 아니라 `sensor_config`, `slam_toolbox`, `beyond_server` 섹션도 포함해야 합니다.

## 전체 Nav2 설정의 의미

이 파일은 검증 이미지의 Nav2 1.3.13 기본 YAML 전체 섹션을 유지한 것입니다. 모든 플러그인의 선택적 옵션을 빠짐없이 열거한 API 명세라는 뜻은 아닙니다. route/docking 설정도 유지되지만 실제 경로 그래프나 도킹 장치 구성이 자동 완성되는 것은 아닙니다. 출처는 [구조와 출처](upstream.md)를 참고하세요.
