#!/usr/bin/env python3
"""
bag2ohm.py — convert a ROS 2 bag of ZED output into OHM inputs.

Reads a rosbag2 (.mcap) containing:
    /zed/zed_node/point_cloud/cloud_registered   (sensor_msgs/PointCloud2)
    /zed/zed_node/pose                            (geometry_msgs/PoseStamped)
    /tf, /tf_static                               (tf2_msgs/TFMessage)

Writes:
    <out>.laz        — concatenated points in the 'map' frame, GpsTime per point.
    <out>_traj.txt   — one line per pose: t x y z qw qx qy qz.

Run on the host with ROS 2 Jazzy sourced.
    pip install laspy lazrs numpy
    source /opt/ros/jazzy/setup.bash
    python3 bag2ohm.py /path/to/bag_dir -o /path/to/run1

The LAZ + trajectory are then fed to ohmpopcuda inside docker-ohm-ros2.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer, TransformException
except ImportError as exc:
    sys.exit(f"ROS 2 import failed ({exc}). Did you `source /opt/ros/jazzy/setup.bash`?")

try:
    import laspy
except ImportError:
    sys.exit("laspy missing — run: pip install laspy lazrs")

try:
    import lazrs  # noqa: F401
    _HAVE_LAZ_BACKEND = True
except ImportError:
    try:
        import laszip  # noqa: F401
        _HAVE_LAZ_BACKEND = True
    except ImportError:
        _HAVE_LAZ_BACKEND = False

CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered"
POSE_TOPIC = "/zed/zed_node/pose"
TARGET_FRAME = "map"

# PointField.datatype → numpy dtype
_DT = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


def open_reader(bag_path: str, storage_id: str = "mcap"):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def iter_messages(reader):
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg_cls = get_message(type_map[topic])
        yield topic, deserialize_message(data, msg_cls), t_ns


def parse_pointcloud2(msg) -> np.ndarray:
    """Return (N, 3) float32 xyz, NaN/Inf dropped, in the message's frame."""
    dtypes = []
    last = 0
    for f in sorted(msg.fields, key=lambda f: f.offset):
        if f.offset > last:
            dtypes.append((f"pad{last}", np.uint8, f.offset - last))
        dtypes.append((f.name, _DT[f.datatype]))
        last = f.offset + np.dtype(_DT[f.datatype]).itemsize
    if msg.point_step > last:
        dtypes.append((f"pad{last}", np.uint8, msg.point_step - last))
    arr = np.frombuffer(bytes(msg.data), dtype=np.dtype(dtypes))
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (qy * qy + qz * qz),     2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),     2 * (qy * qz - qx * qw)],
        [    2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag", help="Path to rosbag2 directory.")
    ap.add_argument("-o", "--output", default="run1",
                    help="Output prefix. Writes <prefix>.laz and <prefix>_traj.txt")
    ap.add_argument("--storage", default="mcap", help="rosbag2 storage plugin (default: mcap)")
    ap.add_argument("--range-min", type=float, default=0.5)
    ap.add_argument("--range-max", type=float, default=5.0)
    ap.add_argument("--voxel", type=float, default=0.0,
                    help="Voxel downsample size in metres. 0 disables.")
    ap.add_argument("--tf-cache-sec", type=float, default=600.0)
    ap.add_argument("--cloud-topic", default=CLOUD_TOPIC)
    ap.add_argument("--pose-topic", default=POSE_TOPIC)
    ap.add_argument("--target-frame", default=TARGET_FRAME)
    args = ap.parse_args()

    out_laz = Path(args.output)
    want_laz = out_laz.suffix.lower() == ".laz" or (out_laz.suffix == "" and _HAVE_LAZ_BACKEND)
    if out_laz.suffix.lower() not in (".laz", ".las"):
        out_laz = out_laz.with_suffix(".laz" if _HAVE_LAZ_BACKEND else ".las")
    elif want_laz and not _HAVE_LAZ_BACKEND:
        print("WARNING: requested .laz but no lazrs/laszip backend installed — writing .las instead.",
              file=sys.stderr)
        print("         Fix: pip install lazrs   (then re-run to produce .laz)", file=sys.stderr)
        out_laz = out_laz.with_suffix(".las")
    out_trj = Path(str(out_laz.with_suffix("")) + "_traj.txt")
    out_laz.parent.mkdir(parents=True, exist_ok=True)

    # Pass 1: fill the TF buffer and record poses.
    print(f"==> Pass 1: scanning {args.bag} for TF + poses")
    tf_buf = Buffer(cache_time=Duration(seconds=args.tf_cache_sec))
    poses: list[tuple[float, float, float, float, float, float, float, float]] = []
    reader = open_reader(args.bag, args.storage)
    for topic, msg, _ in iter_messages(reader):
        if topic == "/tf":
            for tr in msg.transforms:
                tf_buf.set_transform(tr, "bag")
        elif topic == "/tf_static":
            for tr in msg.transforms:
                tf_buf.set_transform_static(tr, "bag")
        elif topic == args.pose_topic:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, o = msg.pose.position, msg.pose.orientation
            poses.append((t, p.x, p.y, p.z, o.w, o.x, o.y, o.z))
    print(f"    {len(poses)} poses collected")

    if not poses:
        sys.exit(f"No messages on {args.pose_topic}. Check the topic name and the bag contents.")

    # Pass 2: project clouds into the target frame.
    print(f"==> Pass 2: transforming clouds into '{args.target_frame}'")
    reader = open_reader(args.bag, args.storage)
    chunks_xyz: list[np.ndarray] = []
    chunks_time: list[np.ndarray] = []
    n_clouds = 0
    n_skipped = 0
    for topic, msg, _ in iter_messages(reader):
        if topic != args.cloud_topic:
            continue
        stamp = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
        try:
            tr = tf_buf.lookup_transform(
                args.target_frame, msg.header.frame_id, stamp,
                timeout=Duration(seconds=0.0),
            )
        except TransformException as e:
            n_skipped += 1
            if n_skipped <= 3:
                print(f"    skip cloud @ {stamp.nanoseconds*1e-9:.3f}s: {e}", file=sys.stderr)
            continue

        xyz_cam = parse_pointcloud2(msg)
        r = np.linalg.norm(xyz_cam, axis=1)
        xyz_cam = xyz_cam[(r >= args.range_min) & (r <= args.range_max)]
        if xyz_cam.size == 0:
            continue

        t_xlate = tr.transform.translation
        q = tr.transform.rotation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        xyz_map = xyz_cam.astype(np.float64) @ R.T + np.array([t_xlate.x, t_xlate.y, t_xlate.z])
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        chunks_xyz.append(xyz_map)
        chunks_time.append(np.full(xyz_map.shape[0], t_sec, dtype=np.float64))
        n_clouds += 1
        if n_clouds % 50 == 0:
            total = sum(c.shape[0] for c in chunks_xyz)
            print(f"    {n_clouds} clouds, {total} points")

    if n_skipped:
        print(f"    ({n_skipped} clouds skipped — TF lookup failed)")
    if not chunks_xyz:
        sys.exit("No usable clouds — check frame ids, TF tree, and range bounds.")

    xyz = np.concatenate(chunks_xyz, axis=0)
    tsec = np.concatenate(chunks_time, axis=0)
    print(f"==> merged: {xyz.shape[0]} points from {n_clouds} frames")

    if args.voxel > 0.0:
        print(f"==> voxel downsample @ {args.voxel} m")
        ijk = np.floor(xyz / args.voxel).astype(np.int64)
        _, first = np.unique(ijk, axis=0, return_index=True)
        xyz, tsec = xyz[first], tsec[first]
        print(f"    kept {xyz.shape[0]} points")

    # LAS/LAZ writing. PF1 = includes GpsTime, no RGB.
    # GpsTime is float64 but we subtract an epoch to avoid losing sub-ms precision.
    # global_encoding bit 0 = 1 marks the time field as Standard GPS Time (not Week Seconds),
    # which PDAL reads as Dimension::Id::GpsTime — OHM's slamio requires this.
    epoch = float(tsec.min())
    print(f"==> writing {out_laz}")
    hdr = laspy.LasHeader(version="1.4", point_format=1)
    hdr.scales = np.array([0.001, 0.001, 0.001])
    hdr.offsets = xyz.min(axis=0)
    hdr.global_encoding.gps_time_type = laspy.header.GpsTimeType.STANDARD
    las = laspy.LasData(hdr)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    las.gps_time = tsec - epoch
    las.write(str(out_laz))

    print(f"==> writing {out_trj}")
    with out_trj.open("w") as f:
        f.write(f"# t x y z qw qx qy qz    (epoch offset subtracted: {epoch:.6f})\n")
        for t, x, y, z, qw, qx, qy, qz in sorted(poses):
            f.write(
                f"{t - epoch:.9f} {x:.6f} {y:.6f} {z:.6f} "
                f"{qw:.9f} {qx:.9f} {qy:.9f} {qz:.9f}\n"
            )

    print("==> done")
    print(f"    ohmpopcuda --cloud {out_laz} --trajectory {out_trj} ...")


if __name__ == "__main__":
    main()
