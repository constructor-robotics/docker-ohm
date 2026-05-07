# docker-ohm

CUDA + ROS 2 Humble container for [CSIRO OHM](https://github.com/csiro-robotics/ohm) — GPU-accelerated probabilistic occupancy / NDT-OM voxel mapping.

Subscribes to a `sensor_msgs/PointCloud2` (from `docker-depth-anything`) plus TF (from `docker-mocap4ros2-optitrack` + `docker-robot-setup-optitrack`) and builds a live 3-D occupancy map.

## Topics / services

| Name | Type | Description |
|---|---|---|
| `/depth_anything_v3/points` | `sensor_msgs/PointCloud2` | Input cloud (subscribed) |
| `/ohm/occupied_cloud` | `sensor_msgs/PointCloud2` | Occupied voxel centres, `map` frame |
| `/ohm/save` | `std_srvs/Trigger` | Flush GPU→CPU and write `.ohm` + `.ply` |

## TF chain (dynamic map)

```
map
 └─ optitrack              ← docker-mocap4ros2-optitrack
     └─ cubr_rg_01
         └─ base_link
             └─ …
                 └─ irayple_rgb_optical_frame   ← point cloud frame
```

The map grows dynamically as the robot moves — no static TF needed.

## Build & run

```bash
cd docker-ohm
docker compose build        # first time only (~20 min, downloads CUDA + OHM)
docker compose up -d

# inside the container — first time only:
source /opt/ros/humble/setup.bash
cd /root/user_ws
colcon build --symlink-install
source install/setup.bash
```

Subsequent shells auto-source the workspace via `.bashrc`.

## Launch

```bash
# inside the container:
ros2 launch ohm_ros2 ohm_live.launch.py

# or directly from host:
docker exec -it ohm-ros2 bash -lc "ros2 launch ohm_ros2 ohm_live.launch.py"
```

### Key launch arguments

```bash
ros2 launch ohm_ros2 ohm_live.launch.py \
    resolution:=0.1 \
    use_ndt:=true \
    range_min:=0.3 \
    range_max:=8.0
```

| Param | Default | Notes |
|---|---|---|
| `resolution` | 0.1 | Voxel size in metres |
| `range_min` / `range_max` | 0.3 / 8.0 | Clip depth cloud (monocular depth is unreliable at extremes) |
| `hit_probability` / `miss_probability` | 0.7 / 0.45 | Log-odds update weights |
| `use_ndt` | true | NDT-OM mode; false = plain occupancy |
| `ndt_sensor_noise_m` | 0.15 | NDT sensor noise (monocular depth is noisier than stereo) |
| `publish_rate_hz` | 2.0 | Occupied-cloud republish rate |
| `cloud_topic` | `/depth_anything_v3/points` | Input point cloud topic |

## Save the live map

One service call writes both formats:

```bash
ros2 service call /ohm/save std_srvs/srv/Trigger {}
```

- `/data/ohm_live.ohm` — native OHM map
- `/data/ohm_live.ply` — voxel PLY for CloudCompare / MeshLab

The `/data/` directory is bind-mounted from `../../bags/` on the host.

## View in RViz2

```
Fixed Frame : map
Add → PointCloud2 → /ohm/occupied_cloud
Add → TF
```

## Tuning for monocular depth

Monocular depth (Depth Anything V3) is noisier than stereo.  Recommended starting values:

```yaml
range_min: 0.3
range_max: 8.0
miss_probability: 0.45
use_ndt: true
ndt_sensor_noise_m: 0.15
```

Use the **FLS mask** (`fls_mask:=true` in `docker-depth-anything`) to restrict the input cloud to the sonar field of view — this removes out-of-region clutter before it reaches OHM.

## Layout

```
docker-ohm/
├── Dockerfile               CUDA 12.x + Ubuntu 22.04 + OHM + ROS 2 Humble
├── docker-compose.yml
├── run.sh                   convenience wrapper (GPU + host net)
├── dds_profiles/            CycloneDDS / FastDDS / Zenoh profiles
└── user_ws/src/ohm_ros2/
    ├── config/ohm_live.yaml
    ├── launch/ohm_live.launch.py
    └── src/ohm_live_node.cpp
```

The `user_ws/` tree is bind-mounted — edit source on the host and `colcon build` inside the container; no image rebuild needed.

## DDS

Profile files are in `dds_profiles/` (CycloneDDS loopback + large receive buffer).  
Set `RMW_IMPLEMENTATION` via the environment or in `run.sh`.  All other containers in this workspace use `rmw_cyclonedds_cpp` — this container must match.
