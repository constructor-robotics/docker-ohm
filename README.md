# docker-ohm-ros2

All-in-one CUDA + ROS 2 Humble container for [CSIRO OHM](https://github.com/csiro-robotics/ohm)
(GPU-accelerated probabilistic voxel occupancy mapping). Contains **both**:

- **Live mapping node** — `ohm_live_node` subscribes to a `sensor_msgs/PointCloud2`
  plus TF, integrates rays into `ohm::GpuMap` on the GPU, publishes the
  occupied-voxel cloud on `/ohm/occupied_cloud` for RViz.
- **Offline OHM CLI tools** — `ohmpop`, `ohmpopcuda`, `ohminfo`, `ohm2ply`,
  `ohmheightmap`, `ohmhm2img`, `ohmfilter`, `ohmprob`, etc. — the full OHM
  toolchain is installed to `/usr/local/bin`. See [USAGE.md](USAGE.md) for the
  CLI guide and [ZED_PIPELINE.md](ZED_PIPELINE.md) for the ZED-specific
  offline (bag → LAZ → `ohmpop`) workflow.

Pairs with [`docker-zed/`](../docker-zed) at runtime for live mapping (it
publishes the cloud + pose + TF).

## What you get

**Live (ROS 2):**

- `ohm_live_node` — subscribes to `/zed/zed_node/point_cloud/cloud_registered`,
  looks up `map ← camera` via TF, builds origin/sample ray pairs, calls
  `ohm::GpuMap::integrateRays()` each frame.
- `/ohm/occupied_cloud` (`sensor_msgs/PointCloud2`) — occupied voxel centres
  in the `map` frame, republished at `publish_rate_hz` (2 Hz default).
- `/ohm/save` (`std_srvs/Trigger`) — flushes GPU→CPU and writes the current
  map as `.ohm` to `save_path` (default `/data/ohm_live.ohm`).

**Offline (CLI):**

- `ohmpopcuda --cloud ... --trajectory ... --output ...` — build a `.ohm`
  from a post-SLAM point cloud + trajectory.
- `ohminfo /data/map.ohm` — inspect a saved map.
- `ohm2ply /data/map.ohm /data/map.ply --voxel-mode voxel` — export to PLY.
- `ohmheightmap` / `ohmhm2img` — 2.5D heightmap + PNG render.
- `bag2ohm.py` — helper that converts a ROS 2 bag of ZED output into a
  `.laz` + trajectory pair for `ohmpop`. Runs on the host (needs ROS 2
  sourced), not inside the container. See its `--help`.

## Map mode — occupancy now, NDT-OM later

The ray mapper is owned as a `std::unique_ptr<ohm::GpuMap>`. `GpuNdtMap`
inherits from `GpuMap`, so switching modes is a parameter:

```bash
ros2 launch ohm_ros2 ohm_live.launch.py use_ndt:=true
```

The map is always constructed with `MapFlag::kVoxelMean` so the switch is
runtime-only — no rebuild, no new Docker image. For NDT-TM or decay-rate,
add the relevant `MapFlag` / mapper and a parameter in
[src/ohm_ros2/src/ohm_live_node.cpp](user_ws/src/ohm_ros2/src/ohm_live_node.cpp).

## Build

```bash
cd ~/cub_marine/optitrack-roboticslab-ws/docker-ohm-ros2
./build.sh                       # ohm-ros2:humble
#   JOBS=8 ./build.sh            # override parallel build jobs
#   ./build.sh v0.5.0            # pin OHM to a tag/commit
```

First build downloads CUDA + Jammy + ROS 2 desktop + OHM — ~10 GB image,
~20 minutes on a fast machine. OHM itself takes the bulk.

## Run — live mapping

Always needs the ZED driver running (on the host or in `docker-zed`) and
publishing `/cloud_registered`, `/pose`, `/tf`, `/tf_static`. Verify with
the checks in [ZED_PIPELINE.md](ZED_PIPELINE.md) section 5 before launching.

```bash
cd ~/cub_marine/optitrack-roboticslab-ws/docker-ohm-ros2
./run.sh                                # interactive shell (GPU + host net)

# --- inside the container (first time only) ---
source /opt/ros/humble/setup.bash
cd /root/user_ws
colcon build --symlink-install
source install/setup.bash

# --- launch the node ---
ros2 launch ohm_ros2 ohm_live.launch.py
```

Subsequent shells auto-source `/root/user_ws/install/setup.bash` via
`.bashrc`, so `ros2 launch ohm_ros2 ohm_live.launch.py` works immediately.

### Auto-launch form

```bash
./run.sh ros2 launch ohm_ros2 ohm_live.launch.py
```

This skips the interactive shell and goes straight to the node (assumes you
already built the workspace at least once).

## Run — offline CLI tools

The same image has the full OHM toolchain. Drop inputs into `/data`
(bind-mounted from `../../bags`) and run the tools directly:

```bash
./run.sh                              # interactive shell
# --- inside the container ---
ohmpopcuda \
    --cloud       /data/scan.laz \
    --trajectory  /data/trajectory.txt \
    --output      /data/out/map \
    --resolution  0.1 \
    --voxel-mean \
    --batch-size  4096

ohminfo  /data/out/map.ohm
ohm2ply  /data/out/map.ohm /data/out/map.ply --voxel-mode voxel
```

Or one-shot without an interactive shell:

```bash
./run.sh ohmpopcuda --cloud /data/scan.laz --trajectory /data/traj.txt \
    --output /data/out/map --resolution 0.1
```

Full tool reference and tuning tips: [USAGE.md](USAGE.md).

## View in RViz

On the host (or any peer on the same ROS_DOMAIN_ID), once the live node is
running:

```bash
rviz2
# Fixed Frame: map
# Add → PointCloud2 → /ohm/occupied_cloud
# Add → TF
```

The cloud updates at `publish_rate_hz`. Walk the ZED through a space, the
occupied voxel set grows.

## Save the live map

```bash
# inside the container (or from any peer with ros2 CLI)
ros2 service call /ohm/save std_srvs/srv/Trigger {}
```

One call produces **two** files side-by-side at `save_path` (default
`/data/ohm_live.ohm`, which is the shared bag directory bind-mounted into
this container):

- `/data/ohm_live.ohm` — native OHM map, written via `ohm::save()`.
- `/data/ohm_live.ply` — voxel PLY for CloudCompare / MeshLab / Open3D.
  Produced by chaining `ohm2ply --voxel-mode voxel` internally after the
  save succeeds. The `.ply` path is derived from `save_path` by swapping
  the extension.

The `Trigger` response `message` reports both paths on success, or the
`.ohm` path plus an `ohm2ply` error if the export step failed (the `.ohm`
is always safe on disk first). For ad-hoc inspection of a saved map,
`ohminfo` is still available in the same container:

```bash
./run.sh ohminfo /data/ohm_live.ohm
```

## Tuning knobs (live node)

All parameters live in
[`user_ws/src/ohm_ros2/config/ohm_live.yaml`](user_ws/src/ohm_ros2/config/ohm_live.yaml)
and can be overridden on the launch command line:

```bash
ros2 launch ohm_ros2 ohm_live.launch.py \
    resolution:=0.05 \
    use_ndt:=true
```

| Param | Default | Why you might change it |
|---|---|---|
| `resolution` | 0.1 | Finer map = more GPU RAM + slower sync. ZED accuracy doesn't reward <0.05. |
| `range_min` / `range_max` | 0.5 / 5.0 | Stereo-noise quadratic with range. >5 m = phantom obstacles. |
| `hit_probability` / `miss_probability` | 0.7 / 0.4 | Widen gap if walls stay fat or free space doesn't clear. |
| `publish_rate_hz` | 2.0 | Raising it costs GPU→CPU sync latency per tick. |
| `use_ndt` | false | true → NDT-OM. Helps with stereo erosion on thin surfaces. |
| `ndt_sensor_noise_m` | 0.05 | Only used when `use_ndt` is true. Start at 0.05, raise for noisier stereo. |
| `expected_element_count` | 16384 | Hint to the GPU cache. Raise if you send >16k points per frame. |
| `gpu_mem_size_mb` | 0 (auto) | Override the OHM GPU cache cap if you run out of memory. |

## Layout

```
docker-ohm-ros2/
├── Dockerfile               CUDA 12.2 + Ubuntu 22.04 + OHM + ROS 2 Humble
├── docker-compose.yml       host net + GPU passthrough
├── build.sh  run.sh         thin wrappers
├── USAGE.md                 offline OHM CLI tool guide
├── ZED_PIPELINE.md          ZED 2i → OHM staged pipeline notes
├── bag2ohm.py               rosbag2 → LAZ + trajectory (run on host)
└── user_ws/src/ohm_ros2/
    ├── package.xml
    ├── CMakeLists.txt        find_package(ohm COMPONENTS CUDA)
    ├── config/ohm_live.yaml
    ├── launch/ohm_live.launch.py
    └── src/ohm_live_node.cpp
```

The `user_ws/` tree is bind-mounted into the container at `/root/user_ws`,
so editing the node source from the host and rebuilding with `colcon build`
inside the running container is fine — no image rebuild required for node
changes.

## Troubleshooting

**`ohm::GpuMap failed to initialise`** — CUDA runtime not visible. Confirm
`nvidia-smi` works on the host, the container was launched with `--gpus all`
(default in `run.sh`), and the driver version is compatible with CUDA 12.2.

**`TF map->zed_left_camera_frame failed`** — ZED SLAM not publishing the
`map` frame. Check `pos_tracking_enabled` / `mapping_enabled` in the ZED
launch config, and re-run the frame checks in
[ZED_PIPELINE.md](ZED_PIPELINE.md).

**Fat walls / free space doesn't clear** — OHM defaults are lidar-tuned.
Try `hit_probability:=0.75 miss_probability:=0.35` and/or tighten
`range_max`. Phantom obstacles at range are almost always a `range_max`
problem; drop it to 4.0 m.

**No occupied voxels after a minute** — verify the cloud rate with
`ros2 topic hz /zed/zed_node/point_cloud/cloud_registered` (should be ≥10 Hz)
and that the cloud is in `zed_left_camera_frame` (not `map`).

## Next steps (once this works)

- Add an NDT-TM mapper (pass `NdtMode::kTraversability` and add an
  intensity channel to the rays — `intensities` arg of `integrateRays`).
- Publish a heightmap: swap `ohmheightmap` logic into the timer using
  `ohm::Heightmap`.
- Publish a TSDF: add `MapFlag::kTsdf`, switch to `GpuTsdfMap`, and change
  the "iterate occupied voxels" step to iterate voxels with
  `|signed_distance| < resolution`.
