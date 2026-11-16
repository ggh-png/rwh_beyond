# D435 단독 센서 점검

`ros_runner/config/runners.yaml`의 `d435` 블록에서 시리얼·깊이/컬러 프로파일·RViz 표시 여부를 설정합니다. 현재 runner 설정은 깊이 480x270x6, 컬러 424x240x6, 시리얼 143322072540, RViz 활성화입니다. 직접 launch 실행의 기본값은 640x480x15입니다. D405 설정은 사용하지 않습니다. D435의 컬러 센서는 `rgb_camera.color_profile`로 설정합니다.

runner를 다시 시작하면 `d435` 실행 항목이 추가됩니다. `d435` 활성화 시 카메라와 전용 RViz2만 시작하며 로봇·MoveIt·BT는 시작하거나 중지하지 않습니다. 종료는 같은 runner를 비활성화하면 됩니다. 실행 중인 다른 프로세스가 동일 카메라를 사용하고 있으면 먼저 해당 카메라 실행을 종료해야 합니다.

수동 실행:

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch rwh_beyond_bringup d435.launch.py
```

RViz Fixed Frame은 `d435_link`입니다. 컬러 `/camera/d435/color/image_raw`, 정렬 깊이 `/camera/d435/aligned_depth_to_color/image_raw`, 색상 점군 `/camera/d435/depth/color/points`를 표시합니다. `/camera/d435/color/camera_info`와 정렬 깊이 CameraInfo도 드라이버에서 제공합니다. 모델 필터로 D405를 대신 선택하지 않습니다.

로봇 장착 TF와 MoveIt 카메라 충돌 형상은 제공하지 않습니다. 센서 단독 검사용이므로 기존 파지 입력 `/camera/color/...`로 연결하지 않습니다. 파지에 쓰려면 실제 장착 보정과 토픽 연결이 별도로 필요합니다.

USB 재연결 후 호스트에서는 보이지만 컨테이너에서 Permission denied가 발생하면 기존 장치 마운트가 낡았는지 확인해야 합니다. 컨테이너 재시작은 내부 실행 프로세스를 종료하므로 작업 종료 후 수행합니다.

공식 설정 참고: https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_camera/launch/rs_launch.py


## 기존 컨테이너의 D435 장치 추가

현재 컨테이너는 생성 당시 `/dev/video0`~`5`와 `/dev/media0`만 전달하도록 만들어져 있습니다. D405에 D435를 추가 연결하면 새 장치가 자동으로 전달되지 않으며, 단순 재시작도 장치 목록을 늘려 주지는 않습니다.

카메라 runner를 중지한 후 호스트에서 아래 명령으로 현재 D435의 영상·미디어 장치만 추가 연결할 수 있습니다. 컨테이너나 로봇을 재시작하지 않습니다.

```bash
python3 /home/ggh/wsyoon_data/d435-20261116/attach_d435_devices_host.py
```

컨테이너 재시작 또는 USB 재연결 후 다시 실행해야 합니다. 이는 현재 컨테이너의 임시 마운트를 갱신하는 방법입니다. 컨테이너 생성 설정에는 영구 반영하지 않습니다. 스크립트 원본은 `ros_runner/scripts/attach_d435_devices_host.py`이며 Linux rootless Podman과 open_tree/move_mount를 사용합니다.

2026-11-16 첫 연결에서는 D435가 USB 2.1로 표시됐고, 640x480x15에서 프레임 손상, 저해상도 6fps에서도 프레임 시간 초과가 발생했습니다. USB 3 케이블·직접 연결 상태에서 다시 확인해야 합니다. 장치 인식과 실제 영상 수신은 구분해서 확인합니다.


## 실물 검증 결과

컨테이너 재시작 후 호스트 장치를 추가 전달하고 `/ros_runner/activate_d435`로 실행했습니다. USB 링크는 여전히 USB 2.1(480Mbps)입니다. 저해상도 6fps 설정에서 컬러, 정렬 깊이, CameraInfo, 색상 점군 메시지를 실제 수신했습니다. RViz2의 OpenGL 초기화와 점군 구독도 확인했습니다. 현재 D435 카메라와 RViz2는 runner에서 실행 중입니다. 로봇·MoveIt·BT는 실행하지 않았습니다.

시작/종료(컨테이너):

```bash
ros2 service call /ros_runner/activate_d435 std_srvs/srv/Empty '{}'
ros2 service call /ros_runner/deactivate_d435 std_srvs/srv/Empty '{}'
```

USB 3 연결은 `lsusb -t`에서 5000M 이상인지 확인한 뒤 프로파일을 높여 시험합니다. USB 재연결·컨테이너 재시작 후 영상 장치 경로가 달라지면 호스트의 장치 전달 스크립트를 다시 실행합니다.
