# 문제 해결

## 본체에서 토픽이 나오지 않을 때 {#clearpath}

먼저 Husky에 SSH로 접속합니다. 아래 재시작은 본체 드라이버를 중단했다가 다시 시작하므로 **주행을 멈추고 Nav2 목표를 취소한 상태**에서 수행합니다.

```bash
ssh administrator@172.16.0.1
systemctl list-unit-files 'clearpath-*.service'
systemctl --no-pager --full status clearpath-robot.service
sudo systemctl restart clearpath-robot.service
systemctl --no-pager --full status clearpath-robot.service
systemctl --no-pager --full status clearpath-platform.service
sudo journalctl -u clearpath-robot.service -u clearpath-platform.service -n 100 --no-pager
```

`clearpath-...`는 설명용 생략 표기입니다. 실제로는 목록에서 확인한 정확한 서비스 이름을 사용합니다. `clearpath-robot.service`가 없다면 임의의 이름으로 계속 재시작하지 말고 설치된 unit을 확인합니다.

Clearpath Humble의 `clearpath-robot`은 platform과 sensors의 상위 서비스입니다. 재시작하면 설정을 재생성하고 종속 서비스도 재시작합니다. 본체 드라이버 또는 본체에서 관리하는 센서만 문제가 있다면 설치된 서비스에 한해 각각 재시작할 수 있습니다. [공식 서비스 설명](https://docs.clearpathrobotics.com/docs/ros2humble/ros/config/services/)

```bash
# 본체 구동·odometry 서비스만 재시작
sudo systemctl restart clearpath-platform.service
# 본체에서 관리하는 센서 서비스만 재시작
sudo systemctl restart clearpath-sensors.service
```

**외부 PC에서 실행한 RWH SICK 드라이버는 본체의 clearpath-sensors 재시작 대상이 아닙니다.** 외부 scan만 없다면 아래 센서 점검 절차를 사용합니다.

재시작 후 본체 ROS 환경을 읽고 메시지 수신을 확인합니다.

```bash
source /opt/ros/humble/setup.bash
if [ -f /etc/clearpath/setup.bash ]; then source /etc/clearpath/setup.bash; fi
ros2 topic list -t
ros2 topic echo /a200_0724/platform/odom/filtered --once --qos-reliability best_effort
```

## 본체에서는 보이는데 외부 PC에서는 안 보임

1. 본체 서비스와 외부 PC의 `ROS_DOMAIN_ID`를 비교합니다.
2. localhost 전용 discovery, `CYCLONEDDS_URI`, RMW 설정을 확인합니다.
3. 컨테이너 네트워크, 로봇 LAN 연결, 방화벽과 multicast 통신을 확인합니다.
4. 환경 변경 후 `ros2 daemon stop`을 실행하고 다시 조회합니다.
5. 토픽은 보이지만 데이터가 없다면 `ros2 topic info --verbose`로 타입과 QoS를 확인합니다.

본체가 정상 발행 중이라면 서비스 재시작만 반복해도 외부 PC의 DDS 설정 문제는 해결되지 않습니다. [외부 PC 준비](setup.md#dds)를 참고하세요.

## scan이 없거나 RViz에 보이지 않음

**외부 PC/컨테이너**에서 확인합니다.

```bash
ping -c 3 172.16.0.161
ros2 topic info /a200_0724/scan --verbose
ros2 topic echo /a200_0724/scan --once --qos-reliability best_effort
```

- ping은 네트워크 도달성 점검이며 센서 드라이버 연결 성공을 보장하지 않습니다. 드라이버 터미널의 연결 오류도 확인합니다.
- 단독 `lidar`와 통합 bringup의 센서 드라이버를 중복 실행하지 않습니다.
- scan의 frame이 `cloud`인지, `base_link → cloud` TF가 있는지 확인합니다.
- RViz LaserScan QoS를 발행자와 맞춥니다. `Best Effort`로 수신되는지도 확인합니다.
- 실기에서 `use_sim_time:=true`로 켜면 실제 센서 launch가 생략됩니다.

## TF 오류 / 지도에 로봇이 없음

[TF 확인 명령](interfaces.md)을 사용해 `base_link → cloud`, `odom → base_link`, `map → odom` 순으로 검사합니다. AMCL 운용 시 RViz에서 초기 위치를 지정합니다. 본체와 외부 PC 시계가 어긋나면 TF extrapolation 오류가 발생할 수 있으므로 시간 동기화 상태도 확인합니다.

```bash
timedatectl status
```

본체가 `/tf`로 보내는데 외부 PC가 `/a200_0724/tf`만 구독하는지도 확인합니다. `odom → base_link`가 없다고 정적 변환을 추가하지 않습니다.

## 경로는 나오는데 로봇이 안 움직임

```bash
ros2 lifecycle get /a200_0724/controller_server
ros2 topic info /a200_0724/cmd_vel --verbose
ros2 topic echo /a200_0724/cmd_vel --once
```

현재 목표 수행 중인지 확인하면서 `cmd_vel_nav`, `cmd_vel_smoothed`, `cmd_vel` 순으로 살펴봅니다. 본체의 실제 구독 토픽과 `Twist` 타입, 비상정지·수동/자동 운용 상태를 확인합니다. collision monitor가 scan 미수신이나 충돌 위험 때문에 출력을 막는지도 확인합니다. 원인을 확인하기 위해 collision monitor를 우회하거나 임의의 속도 명령을 보내지는 않습니다.

## 지도 파일 오류

`map_yaml_file`의 YAML과 YAML의 `image` 경로를 함께 확인합니다. runner는 기본 `/root/wsyoon_data/map/map.yaml`을 쓰며, 수동 bringup의 기본 지도와 다릅니다. [지도 저장](operation.md#slam) 후 실행하거나 `RWH_MAP_FILE`을 올바르게 지정합니다.
