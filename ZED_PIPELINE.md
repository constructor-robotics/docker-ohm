# ZED 2i → OHM mapping pipeline

A practical, staged guide for feeding a **ZED 2i stereo camera** into **OHM** (CSIRO's GPU occupancy map) using the two Dockerized setups in this repo (`docker-zed` for the camera, `docker-ohm-ros2` for mapping — same image hosts both the live ROS 2 node and the offline `ohmpop*` tools).

If you're looking for generic OHM tool usage, see [USAGE.md](USAGE.md) instead. This file is specific to the ZED-as-input case.

---

## 0. Honest reality check

**Confirmed from OHM docs** (`ohm/docs/docutils.md`, `docusage.md`):
- `ohmpop` expects a point cloud file (LAS/LAZ/PLY/PCD via PDAL) + a trajectory file, with **per-point timestamps** in the cloud so the sensor origin can be interpolated from the trajectory.
- Docs explicitly state the cloud should be the output of "a SLAM scan after global optimisation and loop closure" — designed around **lidar SLAM output**.
- The OHM C++ library (`OccupancyMap`, `ohm::GpuMap::integrateRays()`) accepts origin/sample ray pairs directly — **no timestamps required** when driven programmatically.

**Engineering judgements (not OHM-documented — be aware):**
- OHM's hit/miss log-odds defaults are tuned for lidar. Stereo noise is ~quadratic in range; ZED 2i depth beyond ~5–6 m is unreliable. You will spend most of your time on **preprocessing and probability tuning**, not on OHM itself.
- There is **no known official CSIRO OHM ROS 2 wrapper**. [Assumption — verify on their GitHub before relying on one.]
- For "just give me a map", ZED's own `mapping/fused_cloud` is simpler. Reach for OHM specifically when you want OHM's heightmap / clearance / NDT features.

---

## 1. Observed facts from the live ZED (ground truth)

From running `ros2 topic info` / `echo` on a working ZED 2i driver:

| Topic | Confirmed type | Confirmed `frame_id` | Fields / notes |
|---|---|---|---|
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | **`zed_left_camera_frame`** (NOT `map`) | `x, y, z, rgb` (all float32). **No per-point timestamp.** |
| `/zed/zed_node/pose` | `geometry_msgs/PoseStamped` | `map` | camera pose in map |
| `/zed/zed_node/odom` | `nav_msgs/Odometry` | `odom` → `zed_camera_link` | camera pose in odom (drifts) |

Key nuance: "cloud_registered" in the ZED wrapper means "registered with RGB colour", **not** "registered to the map frame". The cloud is in the camera's optical frame — you need TF to move it to a fixed frame before mapping.

---

## 2. Minimum topic selection

| Topic | Use? | Why |
|---|---|---|
| `/zed/zed_node/point_cloud/cloud_registered` | ✅ primary cloud | Raw per-frame cloud. `fused_cloud` is already spatially fused — mapping a map doubles errors. |
| `/tf` + `/tf_static` | ✅ required | Cloud is in `zed_left_camera_frame`. TF is how you transform into a fixed frame. |
| `/zed/zed_node/pose` | ✅ use this | Already in `map` frame. Simplest way to get sensor origin per frame. |
| `/zed/zed_node/odom` | ❌ skip | Redundant with `pose`. `pose` uses SLAM-corrected `map`; `odom` drifts. |
| `/zed/zed_node/depth/*` | ❌ skip | Cloud already encodes this. |
| `/zed/zed_node/mapping/fused_cloud` | ❌ skip | Already voxel-fused internally; you'd feed OHM pre-filtered data. |
| `/zed/zed_node/path_*`, `/imu/data` | ❌ skip | Not useful for OHM. |

---

## 3. Live vs offline — ranked

1. **Path A — Offline: bag → LAZ + trajectory → `ohmpop`.** Matches the documented OHM workflow. Fully deterministic. `docker-ohm-ros2` ships PDAL + `ohmpop*` for this.
2. **Path B — Live ROS 2 node using OHM C++ API.** Subscribe to `PointCloud2` / `tf` / `pose`, call `ohm::GpuMap::integrateRays(origins, samples)` per frame. No per-point timestamps needed. Implemented in this repo as the `ohm_live_node` package inside `docker-ohm-ros2/user_ws` — launch with `ros2 launch ohm_ros2 ohm_live.launch.py`.
3. **Path C — Both as stages.** Develop offline (Path A), deploy live (Path B). Both paths share the same container image.

---

## 4. Pipeline architecture

```
                     HOST (Ubuntu + ROS 2 Jazzy + ZED SDK)
  ┌──────────────────────────────────────────────────────────────────┐
  │   zed_node ──▶ /cloud_registered (PointCloud2, camera frame)     │
  │           ──▶ /pose (map frame)  ──▶ /tf ──▶ /tf_static           │
  │                      │                                            │
  │               ros2 bag record ──▶ run1.mcap                       │
  │                      │                                            │
  │               bag2ohm.py (Python, laspy, numpy)                   │
  │                      │                                            │
  │                      ▼                                            │
  │            run1.laz  +  run1_traj.txt   (in ~/cub_marine/bags)    │
  └──────────────────────────────────────────────────────────────────┘
                                  │  bind-mounted into /data
                                  ▼
                     DOCKER (docker-ohm-ros2, CUDA backend)
  ┌──────────────────────────────────────────────────────────────────┐
  │   ohmpopcuda --cloud /data/run1.laz \                             │
  │              --trajectory /data/run1_traj.txt \                   │
  │              --output /data/out/run1                              │
  │                      │                                            │
  │                      ▼                                            │
  │             run1.ohm ─▶ ohminfo   (sanity check)                  │
  │                      ─▶ ohm2ply   (PLY for CloudCompare)          │
  │                      ─▶ ohmheightmap (2.5D ground projection)     │
  └──────────────────────────────────────────────────────────────────┘
```

---

## 5. Frame consistency checks — do these first

```bash
# A. TF graph reaches the cloud's frame.
ros2 run tf2_ros tf2_echo map zed_left_camera_frame

# B. Cloud's frame_id matches what you echoed.
ros2 topic echo /zed/zed_node/point_cloud/cloud_registered --once --field header
#   Expect: frame_id: zed_left_camera_frame

# C. Pose is in map frame and evolves when you move.
ros2 topic echo /zed/zed_node/pose --once --field header
#   Expect: frame_id: map

# D. TF rate keeps up with cloud rate (no stale transforms).
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
ros2 run tf2_tools view_frames   # generates frames.pdf — inspect the tree
evince frames.pdf

# E. Static offsets camera_link → optical frame exist.
ros2 run tf2_ros tf2_echo zed_camera_link zed_left_camera_frame
#   Expect: ~90° rotation (ROS REP-103 optical convention), zero translation.

# F. Cross-check pose vs TF.
ros2 run tf2_ros tf2_echo map zed_camera_link
#   Compare with /zed/zed_node/pose values — should agree.
```

Red flags: no `map` frame (SLAM not enabled — turn on `pos_tracking.pos_tracking_enabled` + `mapping.mapping_enabled` in ZED launch config); split TF tree; pose frozen at origin; transforms lagging >100 ms.

---

## 6. Stereo preprocessing (in order)

Apply each before the next:

1. **Drop NaN/Inf.** ZED clouds have holes. Filter any point where `x/y/z` isn't finite.
2. **Range clip — 0.5 m to 5.0 m.** ZED 2i sweet spot. Drop near-field ego returns and far-field noise. *Single highest-leverage filter.*
3. **Voxel downsample** to ~your OHM voxel size (5–10 cm). Stereo clouds have ~1–2 M points; OHM will choke on raw.
4. **Statistical outlier removal** (Open3D `remove_statistical_outlier`). Cheap; helps in textureless regions.
5. **Confidence filtering.** Enable `/zed/zed_node/confidence/confidence_map` in `common.yaml` if you want it; threshold around 50/100.
6. **Passthrough on height.** Keep points within a reasonable Z band from base_link (`-1.5 … +2.0 m`).
7. **Ground removal — do NOT do this for OHM.** OHM wants ground rays so it can carve free space. Only strip the ground if you're explicitly building an obstacle-only map.

Do steps 1–3 unconditionally. Add 4–6 if the map looks noisy.

---

## 7. Per-point timestamps — workaround

Your ZED cloud has no per-point time (confirmed — fields are `x,y,z,rgb`). `ohmpop` expects per-point timestamps.

**Workaround (accurate, works cleanly):**
- When exporting to LAZ, assign **every point of frame N the header timestamp of frame N** (as the LAS `GpsTime` field).
- Write the trajectory with **one entry per frame** at the same timestamp, giving the camera's pose in `map`.
- `ohmpop`'s interpolation lands exactly on a trajectory sample → correct sensor origin. No approximation, because every point in a frame really *was* captured at that one instant — stereo has no rolling-shutter-in-time effect like lidar.

**Alternative:** skip the file format entirely — use Path B (C++ node calling `GpuMap::integrateRays`).

---

## 8. Staged implementation plan

### Stage A — Verify topics + transforms
Run the §5 checks. Confirm cloud rate ≥ 10 Hz, pose rate ≥ 30 Hz.

### Stage B — Visualise in RViz
```bash
rviz2
# Fixed Frame: map
# Add: PointCloud2 on /zed/zed_node/point_cloud/cloud_registered
# Add: Pose on /zed/zed_node/pose
# Add: TF
```
Walk around a small area. Enable PointCloud2 **Decay Time: 30 s** to confirm temporal consistency. If the cloud smears or drifts, fix SLAM before touching OHM.

### Stage C — Record a bag
```bash
mkdir -p ~/cub_marine/bags/zed_run1 && cd ~/cub_marine/bags/zed_run1
ros2 bag record -s mcap -o run1 \
    /tf /tf_static \
    /zed/zed_node/point_cloud/cloud_registered \
    /zed/zed_node/pose
```
30–60 s of slow, deliberate motion through a small indoor space. Small is good — you'll iterate on this bag many times.

### Stage D — Convert bag → LAZ + trajectory
Run `bag2ohm.py` on the host. Output: `run1.laz` + `run1_traj.txt` in `~/cub_marine/bags/`.

### Stage E — Generate the map (inside docker-ohm-ros2)
```bash
cd ~/cub_marine/optitrack-roboticslab-ws/docker-ohm-ros2
./run.sh                        # GPU passthrough on by default; /data is read-write
# inside the container:
mkdir -p /data/out
ohmpopcuda \
    --cloud          /data/run1.laz \
    --trajectory     /data/run1_traj.txt \
    --output         /data/out/run1 \
    --resolution     0.1 \
    --voxel-mean \
    --ray-length-max 5.0 \
    --batch-size     4096
# Progress is on by default; pass -q / --quiet to suppress.
# `ohmpopcuda --help` shows all real flags.
```

### Stage F — Inspect
```bash
# inside the container
ohminfo /data/out/run1.ohm
ohm2ply /data/out/run1.ohm /data/out/run1.ply --voxel-mode voxel
# on the host
cloudcompare ~/cub_marine/bags/out/run1.ply   # or Open3D / MeshLab
```
Iterate on `--resolution`, `--hit`, `--miss`, `--range-max`. Don't touch the bag — re-run D→F only.

---

## 9. Host vs container split

| Runs on… | What |
|---|---|
| **Host (ROS 2)** | `ros2 bag record`, `bag2ohm.py`. |
| **`docker-zed`** | ZED driver, publishes cloud + pose + TF. |
| **`docker-ohm-ros2`** | Live: `ohm_live_node` subscribing to ZED topics. Offline: `ohmpop*`, `ohminfo`, `ohm2ply`, `ohmheightmap`. |

Both live (Path B) and offline (Path A) tools ship in the same `docker-ohm-ros2` image. The `/data` bind mount carries bags + saved `.ohm` files between containers and the host.

---

## 10. Conversion script sketch (`bag2ohm.py`)

~80 lines of Python, deps `pip install rosbags laspy lazrs numpy`:

```python
# bag2ohm.py — host side
# Reads a rosbag2 mcap with PointCloud2 + PoseStamped + tf,
# transforms each cloud into 'map' frame, and writes:
#   out.laz       — all points, GpsTime = frame stamp
#   out_traj.txt  — timestamp x y z qw qx qy qz   (one line per frame)

from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
import numpy as np, laspy

store = get_typestore(Stores.ROS2_JAZZY)
cloud_topic = '/zed/zed_node/point_cloud/cloud_registered'
pose_topic  = '/zed/zed_node/pose'

# Pass 1: build a time-sorted pose list  (t, x, y, z, qw, qx, qy, qz).
# Pass 2: for each cloud msg:
#   a. nearest pose (or TF-based transform) gives T_map_camera at msg.stamp
#   b. transform points from camera frame to map frame
#   c. drop NaN/Inf, range-clip to [0.5, 5.0] m (euclidean in camera frame)
#   d. append to arrays; record GpsTime = msg.stamp (float seconds)
# Pass 3: write LAZ with scales 0.001 and GpsTime field (float64).
# Pass 4: write the trajectory text file.
```

Keep it simple. Don't write a live ROS 2 ↔ OHM bridge until Stage F gives you a map you trust.

---

## 11. Inspection cheatsheet

```bash
# Types + QoS
ros2 topic info /zed/zed_node/point_cloud/cloud_registered --verbose
ros2 topic info /zed/zed_node/pose --verbose

# Rate
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
ros2 topic hz /zed/zed_node/pose

# Confirm field list (checks for any per-point time field)
ros2 topic echo /zed/zed_node/point_cloud/cloud_registered --once --field fields
#   Seen:  [x, y, z, rgb]  — no 'time' / 'timestamp' / 't'

# Frame ids
ros2 topic echo /zed/zed_node/point_cloud/cloud_registered --once --field header
ros2 topic echo /zed/zed_node/pose --once --field header

# TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map zed_left_camera_frame
ros2 run tf2_ros tf2_echo map zed_camera_link
ros2 run tf2_ros tf2_monitor

# Data volume
ros2 topic bw /zed/zed_node/point_cloud/cloud_registered
```

---

## 12. Deliverables

### Recommended topic set
```
/zed/zed_node/point_cloud/cloud_registered   — cloud source
/zed/zed_node/pose                           — sensor pose (map frame)
/tf, /tf_static                              — frame graph
```

### Node graph (Path A, offline)
```
zed_node ─┬─▶ /cloud_registered ─┐
          ├─▶ /pose               ├─▶ ros2 bag record ─▶ run1.mcap
          └─▶ /tf, /tf_static ────┘                         │
                                                            ▼
                                                     bag2ohm.py (host)
                                                            │
                                                 run1.laz + run1_traj.txt
                                                            │
                                                            ▼
                                               ohmpopcuda (OHM container)
                                                            │
                                                        run1.ohm
                                                            │
                                                   ┌────────┼────────┐
                                                   ▼        ▼        ▼
                                               ohminfo  ohm2ply  ohmheightmap
```

### Implementation checklist

- [ ] §5 frame checks all pass; TF tree is one piece; pose changes when you move.
- [ ] RViz shows a stable accumulated cloud in `map` frame (Stage B).
- [ ] Record a short bag with exactly four topics: `/cloud_registered`, `/pose`, `/tf`, `/tf_static`.
- [ ] Write `bag2ohm.py`: transform to `map`, NaN drop, range clip 0.5–5.0 m, voxel downsample 0.1 m, LAZ + `GpsTime`, trajectory one line/frame.
- [ ] Verify LAZ + trajectory: `pdal info run1.laz` shows `gpstime` min/max covering the trajectory range.
- [ ] Run `ohmpopcuda` with `--resolution 0.1 --voxel-mean --ray-length-max 5.0`.
- [ ] `ohminfo` shows non-zero occupied voxel count; `ohm2ply` output opens cleanly in CloudCompare.
- [ ] Iterate: tune `--resolution`, `--hit`, `--miss`; add `--ndt` once the basics work.
- [ ] *Only then* consider Path B (live node via `GpuMap::integrateRays`).

### Biggest risks / failure points

1. **Frame confusion.** `cloud_registered` is in the camera frame, not `map`. Mitigate: §5 checks, log `frame_id` in your script.
2. **Time drift between topics.** Pose and cloud come on separate topics; nearest-neighbour matching with >20 ms skew plus fast motion → wrong ray origin. Mitigate: move slowly in first bag; add `ApproximateTime` sync later.
3. **Stereo noise at range.** Without `--range-max ≤ 5 m` the map becomes a haze of phantom obstacles. Single most common failure mode.
4. **Voxel-size mismatch.** Downsample to ~voxel resolution before `ohmpop`. Feeding 1 M points into a 10 cm grid wastes time and may blow the GPU cache.
5. **Lidar-tuned hit/miss.** Defaults may under-carve free space on stereo. If walls are "fat" / space isn't cleared, widen the gap (e.g. `--hit 0.7 --miss 0.4`).
6. **Missing `map` frame.** Without ZED positional tracking enabled, no `map` exists — only `odom`, which drifts. Enable `pos_tracking_enabled` + `mapping_enabled` in the ZED launch config.
7. **`GpsTime` precision.** LAS `GpsTime` is a double; some writers default to 32-bit. Use `laspy` with explicit float64 and subtract a bag-epoch offset to avoid precision loss.
8. **Assuming an OHM ROS 2 wrapper exists.** I don't know of one. Don't build your plan on finding one.
9. **`fused_cloud` temptation.** Don't feed ZED's fused cloud into OHM — you'd be mapping a map, compounding errors. Raw `cloud_registered` is the right input.

---

## Where to go next

- Start with Stage A (§5 checks) on your live system.
- When those pass, record a Stage C bag.
- Ask for the full `bag2ohm.py` implementation when ready — the sketch in §10 is the skeleton.
- For generic OHM tool usage (ohmpop flags, heightmap modes, ohm2ply modes) see [USAGE.md](USAGE.md).
- For docker-ohm-ros2 build/run details see [README.md](README.md).
