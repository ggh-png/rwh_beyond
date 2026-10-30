# SSH와 본체 확인

## 1. 로봇 네트워크에 연결

**외부 PC 터미널**에서 Husky에 도달할 수 있는 유선 또는 무선 네트워크에 연결합니다. 고정 IP를 직접 설정한다면 다른 장비와 중복되지 않는 주소를 사용하고 실제 네트워크의 netmask를 확인합니다.

```bash
ping -c 3 172.16.0.1
ssh administrator@172.16.0.1
```

로그인은 장비에 설정된 비밀번호 또는 SSH 키를 사용합니다. 이 저장소에는 비밀번호를 보관하지 않습니다.

## 2. 본체 ROS와 서비스 확인

이후 명령은 **SSH로 들어간 Husky 터미널**에서 실행합니다.

```bash
ls /opt/ros
echo "$ROS_DISTRO"
systemctl list-unit-files 'clearpath-*.service'
systemctl --no-pager --full status clearpath-robot.service
```

`ROS_DISTRO`가 비어 있어도 ROS가 설치되지 않았다는 뜻은 아닙니다. Humble 설치를 확인했다면 다음과 같이 환경을 읽습니다.

```bash
source /opt/ros/humble/setup.bash
if [ -f /etc/clearpath/setup.bash ]; then
  source /etc/clearpath/setup.bash
fi
printenv | grep -E '^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_LOCALHOST_ONLY|CYCLONEDDS_URI|ROS_DISCOVERY_SERVER)='
ros2 node list
ros2 topic list -t
```

서비스의 실제 환경과 로그인 셸 환경이 다를 수 있습니다. 필요한 경우 다음 명령으로 서비스 설정과 환경 파일 경로를 확인합니다.

```bash
systemctl cat clearpath-platform.service
systemctl show clearpath-platform.service -p Environment -p EnvironmentFiles
```

## 3. 본체 데이터 확인

```bash
ros2 topic info /a200_0724/platform/odom/filtered --verbose
ros2 topic echo /a200_0724/platform/odom/filtered --once --qos-reliability best_effort
ros2 topic info /a200_0724/cmd_vel --verbose
```

odometry 메시지의 `header.frame_id`는 `odom`, `child_frame_id`는 `base_link`인지 확인합니다. 명령 토픽은 현재 RWH 설정상 `geometry_msgs/msg/Twist`를 기대합니다. 실제 본체의 구독 토픽과 타입이 다르면 연결 설정을 먼저 수정해야 합니다.

본체에서도 토픽이 나오지 않는다면 [서비스 재시작 절차](troubleshooting.md#clearpath)를 따릅니다. 본체에서는 보이는데 외부 PC에서 보이지 않으면 [외부 PC 통신 설정](setup.md#dds)을 확인합니다.

LiDAR를 외부 PC에서 실행하는 구성에서는 본체 SSH 셸에 scan이 아직 보이지 않을 수 있습니다. 본체 서비스 상태는 우선 odometry로 판단합니다.
