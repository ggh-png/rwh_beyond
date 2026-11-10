> SAM 3 RGBD 입력과 대상 선택을 추가했습니다. 기존 Action 타입이 변경되어 rebuild와
> 서버/클라이언트 재시작이 필요합니다. 최신 운영 방법·승인 상태·제한은
> [SAM 3 객체 분할](../docs/sam3-perception.md)을 참고하세요.

# rwh_beyond_grasp_bridge

Beyond 레포의 ROS 2 Jazzy ↔ GraspGenX ZMQ Bridge. 인터페이스는 기존 `rwh_beyond_msgs`를 공유한다.

`inference.launch.py` starts only the CPU inference Action bridge, without motion.
`grasp_pipeline.launch.py` additionally starts a MuJoCo-only grasp coordinator that
executes MoveIt trajectories when requested by the BT. See [red-box grasp](../docs/red-box-grasp.md). Input is either a segmented metric object cloud or calibrated depth + a registered target-object mask.
Results are expressed in exactly the selected input cloud/depth optical frame and acquisition stamp.
The returned frame is the GraspGenX gripper base (+Z approach, +X closing), not a
calibrated PiPER TCP. Pose scores do not establish IK or collision feasibility.


## 입력과 결과

`/infer_grasps` (`rwh_beyond_msgs/action/InferGrasps`)는 시간이 걸리는
원격 추론을 위한 Action이다. Service 대신 feedback/취소를 제공한다.

| input_mode | 필수 입력 | 처리 |
|---|---|---|
| POINT_CLOUD (0) | object_cloud | 이미 분할된 객체 XYZ를 공식 infer 요청으로 전송 |
| DEPTH_IMAGE (1) | depth_image, camera_info, object_mask | 객체 마스크의 depth를 XYZ로 역투영한 뒤 infer 요청으로 전송 |

공통 입력은 `gripper_name`, `num_grasps`, `top_k`이다. 결과는 `success`,
`message`, `candidates[]`이며 각 후보는 `id`, `PoseStamped pose`, `score`다.
Feedback: `validating_input` → (필요 시 `converting_depth`) →
`waiting_for_inference` → `converting_candidates`.

두 모드의 입력을 동시에 채우지 않는다. RGB 영상은 GraspGenX의 공식
`infer` 입력이 아니므로 모델로 전송하지 않는다. RGB 기반 객체 검출/분할은
상위 인식 노드가 수행하고, 선택한 한 객체의 점군 또는 마스크를 전달한다.

### Depth 입력 계약

- `16UC1`은 millimeter, `32FC1`은 meter로 해석한다.
- `object_mask`는 같은 크기의 `mono8`/`8UC1`, 0=배경, nonzero=선택한 **한 객체**다.
- Depth, 마스크, CameraInfo는 같은 optical frame이어야 한다.
- Depth/마스크 timestamp 차이는 기본 0.1초 이내다. CameraInfo도 동일하며
  timestamp=0인 정적 calibration만 시각 검사를 생략한다.
- Depth는 왜곡이 제거된 pinhole 영상이고 K가 그 영상에 맞아야 한다.
  D가 0이 아니거나 binning/ROI offset이 있으면 거부한다. 정합되지 않은
  RGB 마스크를 그대로 사용하면 안 된다.
- 0/음수/NaN/Inf depth를 제외하고 최소 100개 유효 객체 픽셀이 필요하다.
  2,073,600픽셀을 초과한 이미지는 거부하고, 객체 포인트가 200,000개보다
  많으면 결정적으로 샘플링한다. 두 한도와 sync tolerance는 YAML로 설정한다.

### GPU 없는 로컬 연결 확인

기존 robot ROS 컨테이너에서 실행한다. 이 모드는 **추론하지 않고 합성
후보를 반환**하며, 결과 message는 `MOCK_ONLY`로 표시된다.

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch rwh_beyond_grasp_bridge inference.launch.py mock:=true
```

다른 터미널의 같은 ROS 환경에서:

```bash
python3 - <<'SAMPLE'
import numpy as np
np.save('/tmp/object.npy', np.tile([0.1, 0.2, 0.3], (120, 1)).astype(np.float32))
np.save('/tmp/depth.npy', np.full((12, 12), 1000, np.uint16))
np.save('/tmp/mask.npy', np.ones((12, 12), np.uint8))
SAMPLE
ros2 run rwh_beyond_grasp_bridge infer_cloud /tmp/object.npy --frame camera_depth_optical_frame
ros2 run rwh_beyond_grasp_bridge infer_depth /tmp/depth.npy /tmp/mask.npy \
  --frame camera_depth_optical_frame --fx 100 --fy 100 --cx 6 --cy 6
```

파일 입력 예제는 저장 파일에 전송 시각을 붙인다. 실제 카메라 연동에서는
촬영 시각을 가진 원본 ROS 메시지를 Action Goal에 넣어야 한다.

### 실제 카메라 노드에서 PointCloud Action 요청

```python
from rclpy.action import ActionClient
from rwh_beyond_msgs.action import InferGrasps

client = ActionClient(node, InferGrasps, 'infer_grasps')
# object_cloud: 인식 노드가 선택한 객체의 sensor_msgs/PointCloud2
future = client.send_goal_async(InferGrasps.Goal(
    input_mode=InferGrasps.Goal.POINT_CLOUD,
    object_cloud=object_cloud, gripper_name='piper_hand',
    num_grasps=100, top_k=20))
# node의 executor를 계속 실행하고, accepted handle.get_result_async()로 결과를 받는다.
```

Depth 경로는 `input_mode=InferGrasps.Goal.DEPTH_IMAGE`로 설정하고 원래
`depth_image`, `camera_info`, `object_mask` 메시지를 채운다.

## Install and build on the robot PC

These commands belong inside the existing robot ROS container (or an approved
robot ROS workspace), **never on the external inference host**.

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
rosdep install --from-paths src/rwh_beyond/rwh_beyond_msgs src/rwh_beyond/rwh_beyond_grasp_bridge --ignore-src -r -y
colcon build --packages-select rwh_beyond_msgs rwh_beyond_grasp_bridge
source install/setup.bash
```

Runtime dependencies: rclpy, sensor_msgs_py, NumPy, pyzmq, msgpack,
msgpack-numpy. No torch/CUDA/model dependencies.

## Tunnel and Action server

On the robot PC host, after VPN and SSH key authentication:

```bash
ssh -N -p 5022 -o IdentitiesOnly=yes -i ~/.ssh/id_ed25519 \
  -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:5556:127.0.0.1:5556 ircgroup1@166.104.138.121
```

If ROS runs inside a container, its `127.0.0.1` is a different network namespace.
Run the SSH tunnel **inside that robot container** using a dedicated user-approved
key/ssh-agent, or explicitly provide a reachable robot-host tunnel endpoint using
an approved network setup. Do not expose the GPU-server port beyond localhost.
A host tunnel at localhost is not automatically reachable from a bridge-network
container. This repository does not mount private SSH keys or change networking.

In the ROS environment:

```bash
ros2 launch rwh_beyond_grasp_bridge inference.launch.py
ros2 action info /infer_grasps
ros2 run rwh_beyond_grasp_bridge infer_cloud object.npy \
  --frame camera_depth_optical_frame --gripper piper_hand --num-grasps 100 --top-k 20
```

`object.npy` must contain measured, segmented `(N,3)` XYZ points in meters.
The CLI stamps a previously saved cloud at send time; for time-dependent TF,
production clients should send PointCloud2 with its original acquisition stamp.
Do not feed the full unsegmented scene. Output JSON includes each position,
quaternion (xyzw), score and frame. The Action result additionally preserves the
original stamp in every PoseStamped.

## Behavior and failures

- One in-flight goal per bridge. Additional goals are rejected, not queued.
- PointCloud input cap: 200,000 points / 32 MiB; minimum 100 finite XYZ points.
- All nonfinite input points are removed, including when `is_dense=true` is wrong.
- PointCloud2 field offsets, endian order and padded rows are respected.
- `1 <= top_k <= num_grasps <= max_grasps` (default maximum 1000).
- Default deadline 60 s covers connect/send/receive together. No automatic retry.
- Each request owns its own ZMQ REQ socket in the executor worker thread;
  failed/canceled sockets are closed with LINGER=0.
- Reentrant callbacks with a multithreaded executor allow cancel and busy rejection
  while inference waits. Cancellation does **not** cancel remote GPU computation.
  After a timeout/cancel the serial upstream server may still be busy.
- `INVALID_INPUT`, `TIMEOUT`, `SERVER_ERROR`, `INVALID_RESPONSE`, `NO_CANDIDATES`,
  `CANCELED`, `INTERNAL_ERROR` distinguish failures in `Result.message`.
- Invalid matrices, reflections, bad homogeneous rows, score shape mismatches,
  NaN/Inf, score outside [0,1], wrong gripper or malformed wire responses fail
  the whole request. Empty inference aborts with `NO_CANDIDATES`.
- Quaternion conversion handles 180-degree rotations without trace division.
- No inference result is published as a motion command.

## Tests (CPU only)

```bash
cd /root/ros2_ws
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=93 python3 -m pytest -q src/rwh_beyond/rwh_beyond_grasp_bridge/test
```

For standard colcon reporting, use:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=93 colcon test --packages-select rwh_beyond_grasp_bridge
colcon test-result --test-result-base build/rwh_beyond_grasp_bridge --verbose
```

The integration fixture starts the actual ROS Action bridge with a mock REP
server speaking the official protocol. This validates serialization and ROS
conversion, not GPU inference, VPN forwarding or checkpoint compatibility.

See [deployment README](../deploy/graspgenx-docker/README.md) and
[phase report](../docs/graspgenx-integration.md).


### BT에서 사용하는 로컬 PointCloud 경로

`segment_target` → `build_object_cloud` → `prepare` 순서로 실행한다.
SAM 서버는 마스크를 반환하고, 로봇 PC의 ROS CPU가 Depth/K로 객체 XYZ를 만든다.
`/grasp/object_cloud` 발행 이후 POINT_CLOUD Action으로 GraspGenX에 전달한다.
따라서 BT에서는 RGBD Action을 통해 SAM을 다시 호출하지 않는다. 단독 RGBD Action
클라이언트 인터페이스는 계속 사용할 수 있다. 기본 트리는 관측 자세와 마지막 홈 복귀를 포함한다.
전체 시뮬레이션 실험과 제한은 [검증 문서](../docs/sam3-perception.md)를 참고한다.
