# SAM 3 segmentation server

See [the full operating guide](../../docs/sam3-perception.md) for authentication,
ROS inputs, instance selection, model limitations and validation status.

The dedicated local rootless Podman container stays up independently of the model.

```bash
bash ~/sam3_workspace/deploy/local_podman.sh start
# Only after facebook/sam3 checkpoint access is approved:
bash ~/sam3_workspace/deploy/local_podman.sh download
bash ~/sam3_workspace/deploy/local_podman.sh model-start
# Another terminal:
bash ~/sam3_workspace/deploy/local_podman.sh health
```

The operational copy is `~/sam3_workspace/deploy`. The authoritative sources are
in this repository. After editing server files, rebuild the image and recreate only
this SAM container to pick up changes; keep models/cache/logs bind mounts.
No automatic color/mock fallback is provided. The server is sequential and each
request builds fresh image state; it does not share state across users or images.

Wire format: msgpack + msgpack-numpy. `action=segment`, uint8 HxWx3 `rgb`, `text`,
optional pixel-xyxy `box`, optional pixel `selection_point`, `selection_policy`.
The reply contains binary uint8 `mask`, `score`, `instance_count`, `backend`, `mock`,
and `elapsed` seconds. `action=health` identifies the loaded backend.
Only `unique` (default) and explicit `highest_score` selection policies are accepted.
Text or box is required. A selection point chooses an instance among detected masks;
it is not a native SAM point prompt. No ROS, TCP correction, IK, or motion runs here.

`compose.yaml` is for an authorized Docker GPU server; fill `.env.example` limits and
provide a compatible Python/CUDA/PyTorch base image. It has not been deployed remotely.
The local tested base image contains Python 3.12, torch 2.9.1+cu130, pyzmq,
msgpack-numpy, scipy and Pillow. Arbitrary bare CUDA base images are not sufficient.
