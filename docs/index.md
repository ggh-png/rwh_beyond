# Husky를 외부 PC에서 운용하기

이 문서는 **Husky 본체는 기본 드라이버와 odometry를 담당하고, 외부 ROS 2 Jazzy PC는 센서·SLAM·Nav2를 실행하는 구성**의 사용 가이드입니다. RWH는 Clearpath 본체 서비스를 대체하지 않으며, 외부 PC의 내비게이션 실행과 설정을 제공합니다.

## 장비와 기본값

| 항목 | 현재 구성 |
| --- | --- |
| Husky SSH | `administrator@172.16.0.1` |
| 로봇 namespace | `a200_0724` |
| 본체 ROS | Humble로 추정 — SSH 접속 후 확인 필요 |
| 외부 PC / 컨테이너 | ROS 2 Jazzy |
| SICK LiDAR 주소 | `172.16.0.161` |
| 센서 프레임 | `cloud` |
| 로봇 기준 프레임 | `base_link` |
| 설정 파일 | `rwh_beyond_bringup/params/nav2_params.yaml` |

IP와 namespace는 이 장비의 값입니다. 다른 Husky에 적용할 때는 해당 장비의 설정으로 바꿉니다.

## 처음 사용할 때의 순서

1. [SSH로 접속](connection.md)하여 Clearpath 서비스와 본체 odometry를 확인합니다.
2. [외부 PC의 ROS 통신 환경](setup.md)을 맞춥니다.
3. [TF와 토픽](interfaces.md)을 확인합니다. 본체가 `odom → base_link`를 제공해야 합니다.
4. 지도가 없으면 [SLAM으로 지도를 저장](operation.md#slam)합니다.
5. 저장한 지도로 [Nav2를 실행](operation.md#navigation)하고 RViz에서 초기 위치를 지정합니다.
6. 반복 운용에는 [ros_runner](runner.md)를 사용합니다.

!!! note "검증 범위"
    격리된 환경에서 launch, 합성 scan/TF, runner 모드 전환과 지도 저장을 검증했습니다. 실제 Humble 본체와 Jazzy PC 사이의 DDS 통신 및 실주행을 보장하는 결과는 아닙니다. 실기에서 토픽 타입·QoS·TF와 속도 명령 수신 여부를 확인해야 합니다.

본체 토픽이 사라졌다면 [Clearpath 서비스 재시작](troubleshooting.md#clearpath)부터 확인하세요.
