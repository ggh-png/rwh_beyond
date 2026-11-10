# Local adapter image: existing CUDA/PyTorch layers are reused, never changed in place.
ARG BASE_IMAGE=localhost/rwh-graspgenx:44eb1d7-cu130
FROM ${BASE_IMAGE}
ARG SAM3_COMMIT=f1647e8d0c1d280991f4d1b7d974c2704537fe66
USER 0
RUN git clone https://github.com/facebookresearch/sam3.git /opt/sam3 && cd /opt/sam3 && git checkout ${SAM3_COMMIT} \
 && python -m pip install --no-cache-dir --no-deps torchvision==0.24.1 --index-url https://download.pytorch.org/whl/cu130 \
 && printf 'torch==2.9.1+cu130\ntorchvision==0.24.1+cu130\n' > /tmp/constraints.txt \
 && python -m pip install --no-cache-dir -c /tmp/constraints.txt 'numpy>=1.26,<2' 'timm>=1.0.17,<1.1' ftfy==6.1.1 'iopath>=0.1.10' einops 'setuptools<81' \
 && python -m pip install --no-cache-dir --no-deps /opt/sam3
RUN python -m pip install --no-cache-dir pycocotools hydra-core psutil
COPY . /opt/segmentation
ENV PYTHONPATH=/opt/sam3:/opt/segmentation SAM3_CHECKPOINT=/data/models/sam3.pt
ENTRYPOINT ["/usr/bin/tini", "-g", "--"]
CMD ["sleep", "infinity"]
