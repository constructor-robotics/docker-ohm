# Using OHM — a practical guide

This guide walks through **what OHM does, how to use the command-line tools**, and how to script against the C++ library. For "how do I build/run the container?", see [README.md](README.md).

Everything below assumes you're **inside the container** (after `./run.sh`).

---

## What OHM builds

OHM is a **probabilistic voxel occupancy map**. You give it:

1. A **point cloud** — the output of a SLAM run after loop closure / global optimisation. Each point must have a timestamp.
2. A **sensor trajectory** — where the lidar was at each timestamp.

OHM traces a ray from the sensor to every point. Voxels the ray passes through get *less* likely to be occupied; voxels where rays terminate get *more* likely. The result is a dense 3D map where every voxel is one of:

- **occupied** — something was hit here
- **free** — rays passed through
- **unknown** — never observed

On top of the occupancy layer you can add **voxel mean** (sub-voxel centroid of hits), **NDT covariance** (surfel-like ellipsoids per voxel), **clearance** (distance to nearest occupied), and a **heightmap** (2.5D ground projection).

OHM's speed edge comes from doing all of this on GPU — CUDA or OpenCL.

---

## The tool chain at a glance

```
┌─────────────┐     ┌─────────────┐     ┌──────────────┐     ┌──────────┐
│ point cloud │──┐  │             │     │              │     │          │
│   (.laz,    │  ├──▶│   ohmpop   │────▶│  map.ohm     │────▶│ ohm2ply  │──▶ .ply
│   .ply,     │  │  │ [cpu|cuda] │     │ (voxel map)  │     │          │
│   .las,...) │  │  │            │     │              │     └──────────┘
└─────────────┘  │  └─────────────┘     │              │     ┌────────────┐
┌─────────────┐  │                      │              │────▶│ohmheightmap│──▶ heightmap.ohm
│ trajectory  │──┘                      │              │     └────────────┘
│  (text/ply) │                         │              │     ┌──────────┐
└─────────────┘                         │              │────▶│ ohminfo  │──▶ stdout
                                        └──────────────┘     └──────────┘
```

Binaries installed inside the container (all on `PATH`):

| Tool | Purpose |
| --- | --- |
| `ohmpopcpu` | Generate a `.ohm` map on CPU. |
| `ohmpopcuda` | Same, CUDA accelerated (our container is built with `OHM_FEATURE_CUDA=ON`). |
| `ohmpopocl` | Same, OpenCL. Present only if the OpenCL feature was enabled at build. |
| `ohminfo` | Print metadata about a `.ohm` file. |
| `ohm2ply` | Export a `.ohm` file to a `.ply` point/voxel cloud. |
| `ohmheightmap` | Flatten a `.ohm` voxel map into a heightmap. |
| `ohmhm2img` | Render a single-layer heightmap as a PNG. |
| `ohmfilter` | Filter an input cloud against an existing map (keep points in occupied voxels). |
| `ohmprob` | Convert between occupancy probability ↔ log-odds value. |
| `ohmquery[cuda,ocl]` | Experimental line/nearest-neighbour queries. |

All accept `--help`.

---

## Input formats

### Point cloud

Because this container is built with `OHM_FEATURE_PDAL=ON`, `ohmpop` accepts anything PDAL can read: `.las`, `.laz`, `.ply`, `.pcd`, `.e57`, etc. The cloud **must contain per-point timestamps** (e.g. the `GpsTime` field in LAS).

### Trajectory

Two options:

1. **Text file** (typical for dumped SLAM trajectories). One sensor pose per line:
    ```
    timestamp x y z qw qx qy qz [extra fields ignored]
    ```
    An optional first header line is allowed. `ohmpop` auto-detects it. Quaternion fields exist but are unused — only position matters for ray tracing.

2. **Point cloud file** (same formats as above) where each point is a sensor pose instead of a lidar return. Useful when your SLAM stack dumps everything as clouds.

For each sample in the input cloud, `ohmpop` finds the two trajectory entries bracketing that timestamp and interpolates the sensor position.

Put both files under `/data` (i.e. `~/cub_marine/bags` on the host — that's what `run.sh` mounts).

---

## Workflow 1 — build a 3D occupancy map

```bash
# inside the container
ohmpopcuda \
    --cloud /data/scan.laz \
    --trajectory /data/trajectory.txt \
    --output /data/out/map \
    --resolution 0.1 \
    --voxel-mean \
    --batch-size 4096
```

What each flag does:

| Flag | Meaning |
| --- | --- |
| `--cloud` | Path to the input point cloud. |
| `--trajectory` | Path to the trajectory text file (or a .ply/.las of sensor poses). |
| `--output` | Output prefix. `ohmpop` writes `<prefix>.ohm` (the map) and `<prefix>.ply` (preview cloud). |
| `--resolution` | Voxel edge length, metres. `0.1` = 10 cm. Trade-off: finer = more memory + slower. |
| `--voxel-mean` | Also store a per-voxel centroid layer (`VoxelMean`). Nice for rendering. |
| `--batch-size` | Rays per GPU batch. 2048/4096 is the sweet spot. |
| `--ndt` | *Optional.* Enable NDT (adds covariance/surfel layer). Slower but much richer. |
| `--clearance <r>` | *Optional.* Compute distance-to-nearest-obstacle up to `r` metres. Useful for planning. |
| `--range-min/--range-max` | Clip rays by distance to reject far outliers or near-field ego returns. |
| `--progress` | Progress bar. |

Want the CPU build instead? Swap `ohmpopcuda` → `ohmpopcpu`. Same arguments.

### Inspect what you produced

```bash
ohminfo /data/out/map.ohm
```

Shows: voxel resolution, chunk size, layer list (occupancy, mean, covariance…), bounding box, occupied / free voxel counts, generation parameters.

### Visualise

`ohmpop` already wrote a preview `.ply`, but `ohm2ply` gives you control:

```bash
# default — one point per occupied voxel, using the voxel-mean position
ohm2ply --input /data/out/map.ohm --output /data/out/map_occupied.ply

# cubes instead of points (nicer in CloudCompare / MeshLab)
ohm2ply --input /data/out/map.ohm --output /data/out/map_cubes.ply --voxel-mode voxel

# NDT covariance surfels — only meaningful if you used --ndt
ohm2ply --input /data/out/map.ohm --output /data/out/map_ndt.ply --mode covariance

# clearance field (needs --clearance at ohmpop time)
ohm2ply --input /data/out/map.ohm --output /data/out/clearance.ply --mode clearance
```

Open the resulting `.ply` in **CloudCompare**, **MeshLab**, or **Open3D**. If you have X11 forwarding on (`run.sh` enables it), you can install and run CloudCompare inside the container:

```bash
sudo apt update && sudo apt install -y cloudcompare
cloudcompare /data/out/map_cubes.ply
```

---

## Workflow 2 — generate a heightmap (2.5D ground projection)

Useful for ground robots: collapse the 3D voxel map into a per-(x,y) height surface.

```bash
# single-layer planar heightmap
ohmheightmap \
    --input /data/out/map.ohm \
    --output /data/out/height \
    --mode planar \
    --ground 0.0

# multi-layer (e.g. ground floor + second floor + roof)
ohmheightmap \
    --input /data/out/map.ohm \
    --output /data/out/height_multi \
    --mode layered
```

Key flags:

| Flag | Meaning |
| --- | --- |
| `--mode` | `planar` (simple band search), `fill` (flood fill from a seed — good for cluttered indoor), `layered`/`layered-unordered` (multi-storey). |
| `--ground` | Initial Z guess for the ground plane (metres). |
| `--clearance` | Minimum vertical free space above ground required to accept a cell. |
| `--search-up / --search-down` | Vertical band to search around the ground estimate. |

The output is another `.ohm` file with a **heightmap layer** on top of occupancy. To look at it:

```bash
# as a coloured PLY (height-tinted)
ohm2ply --input /data/out/height.ohm --output /data/out/height.ply --mode heightmap

# or meshed (experimental)
ohm2ply --input /data/out/height.ohm --output /data/out/height_mesh.ply --mode heightmap-mesh

# or as a PNG (single-layer heightmaps only)
ohmhm2img --input /data/out/height.ohm --output /data/out/height.png
```

---

## Workflow 3 — filter a point cloud against a map

Useful when you have a reference map and want to reject new points that don't land in known occupied space (e.g. ghosting, dynamic objects).

```bash
ohmfilter \
    --map /data/out/map.ohm \
    --cloud /data/new_scan.laz \
    --output /data/out/clean.laz
```

---

## Tuning tips

**The map is too big / slow:**
- Increase `--resolution` (0.2 m instead of 0.05 m = 64× less memory).
- Cap ray length with `--range-max` so long empty rays don't touch far chunks.

**The map is too sparse / noisy:**
- Enable `--voxel-mean` so points accumulate a sub-voxel centroid.
- Add `--ndt` to use NDT — handles noisy sensors much better than vanilla occupancy.
- Tune the occupancy hit/miss probabilities: `--hit 0.7 --miss 0.45` (defaults shown; make the gap wider for stronger confidence).

**Walls look "fat":**
- Lower `--resolution` (finer voxels).
- Use NDT — planar surfaces get tight ellipsoids instead of voxel cubes.

**GPU out-of-memory (CUDA):**
- Drop `--batch-size` to 2048 or 1024.
- Reduce the GPU cache: `--gpu-cache <MB>` (CUDA-specific flag — inspect `--help` on the exact name in your version).

**Runs slower on GPU than CPU:**
- Your `--batch-size` is too small. Start at 4096.
- The cloud is so small the kernel launch overhead dominates. Use `ohmpopcpu` for tiny datasets.

---

## Using OHM from C++ (library mode)

`ohmpop` is a thin wrapper around the `ohm` / `ohmgpu` libraries. Anything `ohmpop` does you can do in your own program. Minimal example for building a map from rays you already have in memory:

```cpp
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperOccupancy.h>
#include <glm/vec3.hpp>

int main()
{
  // 10 cm voxels, default chunk size.
  ohm::OccupancyMap map(0.1);

  // Vanilla CPU ray integrator. For GPU, use ohm::GpuMap (from ohmgpu).
  ohm::RayMapperOccupancy mapper(&map);

  // Rays are flat pairs: [origin, sample, origin, sample, ...].
  std::vector<glm::dvec3> rays = {
      {0, 0, 1},  {1, 0, 1},
      {0, 0, 1},  {1, 1, 1},
  };

  mapper.integrateRays(rays.data(), rays.size());
  return 0;
}
```

For GPU, swap the mapper:

```cpp
#include <ohmgpu/GpuMap.h>

ohm::OccupancyMap map(0.1);
ohm::GpuMap gpu_map(&map);          // wraps the map; also acts as the ray mapper
gpu_map.integrateRays(rays.data(), rays.size());
gpu_map.syncVoxels();               // pull results back to CPU memory before reading
```

Compile against the installed OHM (the container installs to `/usr/local`):

```cmake
find_package(ohm REQUIRED)
find_package(ohmgpu REQUIRED)       # optional
add_executable(mymap main.cpp)
target_link_libraries(mymap PRIVATE ohm::ohm ohm::ohmgpu)
```

The full C++ reference is in:
- [docs/docusage.md](../../ohm/docs/docusage.md) — map usage, layers, iteration, NDT.
- [docs/docvoxellayers.md](../../ohm/docs/docvoxellayers.md) — how to add your own per-voxel data.
- [docs/gpu/](../../ohm/docs/gpu) — GPU algorithm details.

---

## End-to-end example

```bash
# on host — drop data in the mounted folder
cp ~/Downloads/handheld_slam.laz       ~/cub_marine/bags/
cp ~/Downloads/handheld_trajectory.txt ~/cub_marine/bags/

# enter container
cd ~/cub_marine/optitrack-roboticslab-ws/docker-ohm-ros2
./run.sh           # run.sh enables GPU passthrough by default

# inside the container
mkdir -p /data/out

ohmpopcuda \
    --cloud       /data/handheld_slam.laz \
    --trajectory  /data/handheld_trajectory.txt \
    --output      /data/out/handheld \
    --resolution  0.05 \
    --voxel-mean \
    --ndt \
    --batch-size  4096 \
    --progress

ohminfo /data/out/handheld.ohm

ohm2ply       --input /data/out/handheld.ohm      --output /data/out/handheld_voxels.ply --voxel-mode voxel
ohmheightmap  --input /data/out/handheld.ohm      --output /data/out/handheld_height --mode planar
ohm2ply       --input /data/out/handheld_height.ohm --output /data/out/handheld_height.ply --mode heightmap
ohmhm2img     --input /data/out/handheld_height.ohm --output /data/out/handheld_height.png
exit

# back on the host — results are already there via the bind mount
xdg-open ~/cub_marine/bags/out/handheld_voxels.ply
xdg-open ~/cub_marine/bags/out/handheld_height.png
```

---

## When something goes wrong

**`ohmpopcuda: command not found`**
The image was built OpenCL-only. Either use `ohmpopocl` / `ohmpopcpu`, or rebuild with `OHM_FEATURE_CUDA=ON` (already the default in this Dockerfile).

**`CUDA error: no CUDA-capable device detected`**
The container didn't see the GPU. Check that you ran with `GPU=1 ./run.sh` (or `--gpus all` if using raw `docker run`), and that `nvidia-smi` works on the host.

**`Failed to load trajectory`**
Your trajectory's time base doesn't match the cloud. Trajectory timestamps must be in the same clock as the cloud's `GpsTime`. A common fix is to confirm both were exported from the same SLAM run.

**Map comes out empty or tiny**
Almost always a time-alignment problem between cloud and trajectory. Print the first/last timestamp of each — they should overlap. If the SLAM dumped the trajectory in seconds-since-start and the cloud in absolute GPS time, convert one.

**PDAL errors like "Unable to open file"**
PDAL supports dozens of formats via plugins. If you're using something exotic (E57, COPC) and it fails, convert to LAZ first with `pdal translate input.e57 output.laz` (PDAL is installed in the container).

**Heightmap is full of holes**
Your `--clearance` is too strict, or `--search-up`/`--search-down` is too narrow. Loosen them. Try `--mode fill` for indoor spaces.

---

## References

- Paper: Stepanas et al., *OHM: GPU Based Occupancy Map Generation*, IEEE RA-L 2022 — DOI [10.1109/LRA.2022.3196145](https://doi.org/10.1109/LRA.2022.3196145).
- Source + API docs: <https://github.com/csiro-robotics/ohm>
- Upstream util reference: [ohm/docs/docutils.md](../../ohm/docs/docutils.md).
