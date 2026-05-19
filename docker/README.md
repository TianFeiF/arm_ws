# Docker for armv7

Two ways to use Docker with this project.

## 1. Dev container (sim-only)

For development without real hardware. RViz works via X11 forwarding.

```bash
xhost +local:docker          # one-time per session
docker compose -f docker/docker-compose.yml up dev
# in another shell:
docker exec -it armv7-dev bash
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false
```

The container bind-mounts `src/` from your host, so edits in your IDE are reflected inside immediately. You still need to `colcon build` inside the container after changing C++ code.

## 2. Hardware container

Passes through `/dev/EtherCAT0` and sets realtime ulimits. **Host must have IgH master kernel module loaded** — Docker cannot load kernel modules.

```bash
# On host: confirm IgH is running
systemctl is-active ethercat                # active
ls -l /dev/EtherCAT0                        # exists

# Start the hardware container
docker compose -f docker/docker-compose.yml up hardware
docker exec -it armv7-hw bash
ros2 launch armv7_bringup arm.launch.py
```

Inside the container, `id` should show both `ethercat` and `realtime` groups (the image is built with the user in both). The `--cap-add SYS_NICE` + `--ulimit rtprio=99` let `chrt -f 99` succeed without further host config.

## Limitations

- Real-time performance inside Docker is slightly worse than native (extra ns of cgroup overhead). Acceptable at 100 Hz; for 500 Hz+ run natively.
- GPU-accelerated RViz inside Docker requires NVIDIA Container Toolkit. Not configured here.
- DDS multicast across host ↔ container: relies on `network_mode: host`.
- Image size ~3.2 GB.

## Rebuild

```bash
docker compose -f docker/docker-compose.yml build --no-cache dev
```

## Clean up

```bash
docker compose -f docker/docker-compose.yml down
docker image rm armv7:dev
```
