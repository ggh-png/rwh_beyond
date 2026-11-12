# BEYOND Grasp

D405 RGB-D, SAM 3, GraspGenX, ROS 2 Behavior Tree와 MoveIt 2를 연결한
PiPER 물체 인식·파지·리프트 프로젝트입니다.

- 로봇 PC: 센서·TF·점군·ROS bridge·BT·경로 계획·실물 제어.
- GPU 서버: SAM 3 분할과 GraspGenX 파지 후보 생성.
- 심·실물 공통 시나리오: `rwh_beyond_bt/behavior_trees/demo.xml`.
- 요청별 대상 지정: `/run_demo` Action의 `target_text`.

**[프로젝트 문서](https://ggh-png.github.io/rwh_beyond/)** · [문서 소스](docs/index.md)

## 시작하기

1. [환경과 빌드](docs/project-containers.md)
2. [실행 가이드](docs/grasp-pipeline-guide.md)
3. [공통 BT와 설정](docs/real-bt.md)
4. [Action 인터페이스](docs/bt-operation.md)
5. [코드 안내](docs/project-code-map.md)

ROS workspace는 `/root/ros2_ws`, 의존 저장소는 `moma-mj-lab.repos`에서 관리합니다.
기반 ROS·GPU·장치 환경을 준비한 뒤 빌드합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
vcs import --recursive --skip-existing src < src/rwh_beyond/moma-mj-lab.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 실행

GPU 모델 서버와 endpoint를 준비한 뒤 runner를 시작합니다.

```bash
ros2 launch ros_runner runner.launch.py
```

별도 터미널에서 심 또는 실물 중 하나를 활성화합니다.

```bash
# 심
ros2 service call /ros_runner/activate_bt_demo_sim std_srvs/srv/Empty '{}'
# 실물: CAN·모터·제어 모드 준비 후 사용
ros2 service call /ros_runner/activate_bt_demo std_srvs/srv/Empty '{}'
```

대상을 지정해 실행합니다.

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{target_text: "yellow block"}' --feedback
```

기본 순서는 home → 열기 → 관측 → 분할·점군·파지 추론·경로 선택 → 접근 →
닫기·계획용 부착 → 리프트 → 관측 복귀 → home입니다.
BT 공통 YAML은 `rwh_beyond_bringup/config/bt_demo.yaml`, 관측 자세는 `demo.xml`에서 조정합니다.

## 현재 동작 범위

실제 잡힘 여부는 판정하지 않으며 실물 닫기는 4초 명령 완료를 기준으로 합니다.
자동 놓기·Place·Navigation은 포함하지 않습니다. 외부 Navigation 연동에는 RunDemo Action을 제공합니다.
카메라 장착 TF는 명목값이며 hand-eye 실측 보정이 필요합니다.
Action 성공은 시퀀스 완료를 뜻하며 일반 물체의 파지 성공률을 보장하지 않습니다.

## 문서 빌드

```bash
pip install -r requirements-docs.txt
mkdocs build --strict
```

문서 변경은 GitHub Actions에서 빌드해 GitHub Pages에 배포합니다.
