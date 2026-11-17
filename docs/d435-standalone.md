# D435 단독 점검

단독 `d435` runner는 카메라와 전용 RViz만 실행합니다. 로봇·MoveIt·BT는 시작하지 않습니다.
BT에 연결하려면 [D435 BT 연결](d435-bt-demo.md)을 따릅니다.

## 장치 설정과 실행

`ros_runner/config/runners.yaml`의 `d435`에서 시리얼·깊이/컬러 프로파일·RViz를 설정합니다.
현재 설정은 깊이 `480x270x6`, 컬러 `424x240x6`입니다. 시리얼은 사용할 장치에 맞춥니다.
직접 launch의 기본 프로파일은 `640x480x15`로 runner 설정과 다릅니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
# ros_runner가 실행 중인 터미널과 별도로 호출
ros2 service call /ros_runner/activate_d435 std_srvs/srv/Empty '{}'
ros2 service call /ros_runner/deactivate_d435 std_srvs/srv/Empty '{}'
```

직접 launch로 점검할 수도 있습니다. runner와 동시에 실행하지 않습니다.

```bash
ros2 launch rwh_beyond_bringup d435.launch.py \
  depth_profile:=480x270x6 color_profile:=424x240x6
```

## 확인할 데이터

| 데이터 | 단독 실행 토픽 |
|---|---|
| RGB | `/camera/d435/color/image_raw` |
| 정렬 Depth | `/camera/d435/aligned_depth_to_color/image_raw` |
| RGB CameraInfo | `/camera/d435/color/camera_info` |
| 색상 점군 | `/camera/d435/depth/color/points` |

RViz Fixed Frame은 `d435_link`입니다. 이 모드는 로봇 장착 TF와 MoveIt 카메라 형상을
제공하지 않으며 파지용 공통 토픽으로 remap하지 않습니다.

## USB와 컨테이너 장치 전달 { #usb-container-devices }

호스트에서 보이는 장치가 컨테이너에서도 열리는지 확인합니다. 동일 카메라를 두 프로세스가
열지 않도록 하고, `lsusb -t`로 USB 링크 속도를 확인합니다. USB 3 연결은 일반적으로
5000M 이상으로 표시됩니다. 지원 프로파일·전원·케이블은 실제 프레임 수신으로 확인합니다.

기존 rootless Podman 컨테이너에 영상 장치가 누락되거나 USB 재연결 뒤 마운트가 낡았다면,
카메라 runner를 중지한 상태에서 **호스트**의 `ros_runner` checkout에서 실행합니다.

```bash
python3 scripts/attach_d435_devices_host.py --container moma_mj_lab_1.1.2_hostcan
```

스크립트가 컨테이너 안에만 있다면 먼저 호스트로 복사해 실행합니다.
이 도구는 Linux `open_tree/move_mount`로 현재 연결된 D435 영상·미디어 장치를 전달합니다.
컨테이너 생성 설정을 영구 변경하지 않으므로 재연결·재시작 후 다시 필요할 수 있습니다.

[Runner 설정](runner-device-settings.md) · [문제 해결](troubleshooting.md)
