#!/usr/bin/sh

docker build \
  --build-arg OVSHARE_GID=$(getent group ovshare | cut -d: -f3) \
  -t isaac-sim:v1 .

docker run --name isaac-sim -it --rm \
    --entrypoint bash \
    --runtime=nvidia \
    --gpus all \
    --network=host \
    --ipc=host \
    -e "ACCEPT_EULA=Y" \
    -e "PRIVACY_CONSENT=Y" \
    -e DISPLAY=$DISPLAY \
    -v $HOME/.Xauthority:/isaac-sim/.Xauthority \
    -u 1234:1234 \
    -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
    -v ~/.cache/ov/cache/hub:/isaac-sim/.cache/ov/cache/hub:rw \
    -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
    -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
    isaac-sim:v1
