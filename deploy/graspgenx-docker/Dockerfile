# Candidate baseline; validate host driver and GPU access before building remotely.
ARG CUDA_IMAGE=docker.io/nvidia/cuda:13.0.2-runtime-ubuntu24.04
FROM ${CUDA_IMAGE}
ARG GRASPGENX_REF=44eb1d77d6d27bc97ee361e3afb717a2115e30a9
ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1 \
    PATH=/opt/venv/bin:$PATH HF_HOME=/data/cache/huggingface \
    XDG_CACHE_HOME=/data/cache HOME=/tmp
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-venv git git-lfs ca-certificates libgomp1 tini \
    && git lfs install --system && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv /opt/venv \
    && pip install --no-cache-dir pip==25.3 uv==0.9.9 \
    && pip install --no-cache-dir torch==2.9.1 --index-url https://download.pytorch.org/whl/cu130
WORKDIR /opt/GraspGenX
RUN git init . && git remote add origin https://github.com/NVlabs/GraspGenX.git \
    && git fetch --depth 1 origin "${GRASPGENX_REF}" && git checkout --detach FETCH_HEAD \
    && pip install --no-cache-dir '.[serve]' \
    && pip check && mkdir -p ext
# Upstream asset lookup is relative to the checkout, not site-packages.
ENV PYTHONPATH=/opt/GraspGenX
COPY scripts /opt/deploy/scripts
COPY config /opt/deploy/config
EXPOSE 5556
# Compose overrides to the remote user's numeric UID/GID.
USER 65532:65532
ENTRYPOINT ["/usr/bin/tini", "-g", "--", "/bin/bash", "/opt/deploy/scripts/start_server.sh"]
