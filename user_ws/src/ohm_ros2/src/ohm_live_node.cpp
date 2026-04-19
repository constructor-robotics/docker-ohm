// ohm_live_node.cpp — subscribe to ZED PointCloud2 + TF, integrate rays into
// an OHM GPU occupancy map, publish the occupied-voxel cloud for RViz.
//
// Flipping to NDT-OM is one parameter: set `use_ndt` to true. The internal
// ray-mapper pointer is a GpuMap*, which GpuNdtMap inherits from, so the
// per-frame integrateRays() path doesn't change.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ohm/MapFlag.h>
#include <ohm/MapLayout.h>
#include <ohm/MapSerialise.h>
#include <ohm/NdtMode.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayFlag.h>
#include <ohm/Voxel.h>
#include <ohm/VoxelOccupancy.h>
#include <ohmgpu/GpuMap.h>
#include <ohmgpu/GpuNdtMap.h>

namespace
{
constexpr double kMb = 1024.0 * 1024.0;
}  // namespace

class OhmLiveNode : public rclcpp::Node
{
public:
  OhmLiveNode() : Node("ohm_live_node")
  {
    declareParameters();
    buildMap();
    wireRos();
    RCLCPP_INFO(get_logger(),
                "ohm_live_node ready — resolution=%.3f m, range=[%.2f, %.2f] m, "
                "hit=%.2f miss=%.2f, target_frame=%s, mode=%s",
                resolution_, range_min_, range_max_, hit_prob_, miss_prob_,
                target_frame_.c_str(), use_ndt_ ? "NDT-OM" : "Occupancy");
  }

private:
  // ------------------------------------------------------------------ params
  double resolution_{};
  double range_min_{};
  double range_max_{};
  double hit_prob_{};
  double miss_prob_{};
  double tf_lookup_timeout_{};
  double publish_rate_hz_{};
  int expected_element_count_{};
  int gpu_mem_size_mb_{};
  bool use_ndt_{};
  double ndt_sensor_noise_{};
  std::string target_frame_;
  std::string cloud_topic_;
  std::string occupied_cloud_topic_;
  std::string save_path_default_;

  // ------------------------------------------------------------------ state
  std::unique_ptr<ohm::OccupancyMap> occ_map_;
  std::unique_ptr<ohm::GpuMap> ray_mapper_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  size_t rays_integrated_{0};
  size_t frames_seen_{0};
  size_t frames_skipped_tf_{0};

  // ------------------------------------------------------------------ setup
  void declareParameters()
  {
    resolution_ = declare_parameter<double>("resolution", 0.1);
    range_min_ = declare_parameter<double>("range_min", 0.5);
    range_max_ = declare_parameter<double>("range_max", 5.0);
    hit_prob_ = declare_parameter<double>("hit_probability", 0.7);
    miss_prob_ = declare_parameter<double>("miss_probability", 0.4);
    tf_lookup_timeout_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
    expected_element_count_ = declare_parameter<int>("expected_element_count", 16384);
    gpu_mem_size_mb_ = declare_parameter<int>("gpu_mem_size_mb", 0);
    use_ndt_ = declare_parameter<bool>("use_ndt", false);
    ndt_sensor_noise_ = declare_parameter<double>("ndt_sensor_noise_m", 0.05);
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    cloud_topic_ = declare_parameter<std::string>(
        "cloud_topic", "/zed/zed_node/point_cloud/cloud_registered");
    occupied_cloud_topic_ = declare_parameter<std::string>(
        "occupied_cloud_topic", "/ohm/occupied_cloud");
    save_path_default_ = declare_parameter<std::string>(
        "save_path", "/data/ohm_live.ohm");
  }

  void buildMap()
  {
    // NDT-OM needs voxel-mean positions; plain occupancy doesn't, but enabling
    // kVoxelMean costs little and makes the switch seamless at runtime.
    ohm::MapFlag flags = ohm::MapFlag::kDefault | ohm::MapFlag::kVoxelMean;
    occ_map_ = std::make_unique<ohm::OccupancyMap>(resolution_, flags);
    occ_map_->setHitProbability(static_cast<float>(hit_prob_));
    occ_map_->setMissProbability(static_cast<float>(miss_prob_));

    const size_t gpu_mem = gpu_mem_size_mb_ > 0
                              ? static_cast<size_t>(gpu_mem_size_mb_) * static_cast<size_t>(kMb)
                              : 0u;
    const unsigned expected_elements = static_cast<unsigned>(std::max(1, expected_element_count_));

    if (use_ndt_)
    {
      auto ndt = std::make_unique<ohm::GpuNdtMap>(
          occ_map_.get(), /*borrowed_map=*/true, expected_elements, gpu_mem, ohm::NdtMode::kOccupancy);
      ndt->setSensorNoise(static_cast<float>(ndt_sensor_noise_));
      ray_mapper_ = std::move(ndt);
    }
    else
    {
      ray_mapper_ = std::make_unique<ohm::GpuMap>(
          occ_map_.get(), /*borrowed_map=*/true, expected_elements, gpu_mem);
    }

    if (!ray_mapper_->gpuOk())
    {
      throw std::runtime_error("ohm::GpuMap failed to initialise — check CUDA runtime");
    }
  }

  void wireRos()
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS cloud_qos = rclcpp::SensorDataQoS();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, cloud_qos,
        std::bind(&OhmLiveNode::onCloud, this, std::placeholders::_1));

    rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
    pub_qos.reliable();
    pub_qos.durability_volatile();
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(occupied_cloud_topic_, pub_qos);

    save_srv_ = create_service<std_srvs::srv::Trigger>(
        "/ohm/save",
        std::bind(&OhmLiveNode::onSave, this, std::placeholders::_1, std::placeholders::_2));

    const auto period =
        std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&OhmLiveNode::onPublish, this));
  }

  // ------------------------------------------------------------------ cloud
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    ++frames_seen_;
    if (msg->header.frame_id.empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "cloud has empty frame_id — dropping");
      return;
    }

    geometry_msgs::msg::TransformStamped tf_cam_to_map;
    try
    {
      tf_cam_to_map = tf_buffer_->lookupTransform(
          target_frame_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(tf_lookup_timeout_));
    }
    catch (const tf2::TransformException &ex)
    {
      ++frames_skipped_tf_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "TF %s->%s failed (%zu skipped): %s",
                           target_frame_.c_str(), msg->header.frame_id.c_str(),
                           frames_skipped_tf_, ex.what());
      return;
    }

    // Convert to tf2::Transform so we use the library's battle-tested
    // quaternion-to-matrix math and avoid any hand-rolled sign errors.
    tf2::Transform T;
    tf2::fromMsg(tf_cam_to_map.transform, T);

    const tf2::Vector3 origin_tf = T.getOrigin();
    const glm::dvec3 origin(origin_tf.x(), origin_tf.y(), origin_tf.z());

    // Throttled diagnostic: shows the actual TF the node is consuming. If
    // you see the translation jump discretely (>5 cm between log lines),
    // ZED SLAM is teleporting its `map` frame on loop closure.
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s<-%s @ t=%.3f: xyz=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
        target_frame_.c_str(), msg->header.frame_id.c_str(),
        rclcpp::Time(msg->header.stamp).seconds(),
        tf_cam_to_map.transform.translation.x,
        tf_cam_to_map.transform.translation.y,
        tf_cam_to_map.transform.translation.z,
        tf_cam_to_map.transform.rotation.x,
        tf_cam_to_map.transform.rotation.y,
        tf_cam_to_map.transform.rotation.z,
        tf_cam_to_map.transform.rotation.w);

    // Expect x,y,z as float32 — ZED PointCloud2 always matches.
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    const size_t point_count = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    std::vector<glm::dvec3> rays;
    rays.reserve(point_count * 2u);

    const double r_min_sq = range_min_ * range_min_;
    const double r_max_sq = range_max_ * range_max_;

    for (size_t i = 0; i < point_count; ++i, ++it_x, ++it_y, ++it_z)
    {
      const float px = *it_x;
      const float py = *it_y;
      const float pz = *it_z;
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz))
      {
        continue;
      }
      const double sq = double(px) * px + double(py) * py + double(pz) * pz;
      if (sq < r_min_sq || sq > r_max_sq)
      {
        continue;
      }
      const tf2::Vector3 p_map_tf = T * tf2::Vector3(px, py, pz);
      rays.push_back(origin);
      rays.emplace_back(p_map_tf.x(), p_map_tf.y(), p_map_tf.z());
    }

    if (rays.empty())
    {
      return;
    }

    const size_t integrated = ray_mapper_->integrateRays(
        rays.data(), rays.size(), nullptr, nullptr, ohm::kRfDefault);
    rays_integrated_ += integrated;
  }

  // ------------------------------------------------------------------ publish
  void onPublish()
  {
    if (!ray_mapper_ || !occ_map_)
    {
      return;
    }

    ray_mapper_->syncVoxels();

    ohm::Voxel<const float> occupancy(occ_map_.get(), occ_map_->layout().occupancyLayer());
    if (!occupancy.isLayerValid())
    {
      return;
    }

    std::vector<float> xyz;
    xyz.reserve(16384);

    for (auto iter = occ_map_->begin(); iter != occ_map_->end(); ++iter)
    {
      occupancy.setKey(*iter);
      if (!occupancy.isValid() || !ohm::isOccupied(occupancy))
      {
        continue;
      }
      const glm::dvec3 centre = occ_map_->voxelCentreGlobal(*iter);
      xyz.push_back(static_cast<float>(centre.x));
      xyz.push_back(static_cast<float>(centre.y));
      xyz.push_back(static_cast<float>(centre.z));
    }

    publishCloud(xyz);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "frames: %zu (tf-skipped %zu), rays: %zu, occupied voxels: %zu",
                         frames_seen_, frames_skipped_tf_, rays_integrated_,
                         xyz.size() / 3u);
  }

  void publishCloud(const std::vector<float> &xyz)
  {
    const size_t num_points = xyz.size() / 3u;
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = get_clock()->now();
    out.header.frame_id = target_frame_;
    out.height = 1;
    out.is_dense = true;
    out.is_bigendian = false;

    // mod.resize() sets width, row_step, and data size based on point_step
    // that setPointCloud2FieldsByString just established — always call
    // resize() after the field setup, never rely on a pre-set width.
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(num_points);

    const size_t bytes_to_copy = num_points * 3u * sizeof(float);
    if (out.data.size() >= bytes_to_copy)
    {
      std::memcpy(out.data.data(), xyz.data(), bytes_to_copy);
    }
    cloud_pub_->publish(std::move(out));
  }

  // ------------------------------------------------------------------ save
  // Derive a .ply path from a .ohm path by swapping the extension (or
  // appending .ply if the input doesn't end in .ohm).
  static std::string derivePlyPath(const std::string &ohm_path)
  {
    const std::string ext = ".ohm";
    if (ohm_path.size() >= ext.size() &&
        ohm_path.compare(ohm_path.size() - ext.size(), ext.size(), ext) == 0)
    {
      return ohm_path.substr(0, ohm_path.size() - ext.size()) + ".ply";
    }
    return ohm_path + ".ply";
  }

  // Single-quote a string for safe inclusion in a /bin/sh command line.
  // Embedded single quotes are escaped by closing the quote, escaping the
  // quote, and reopening — standard POSIX idiom.
  static std::string shellQuote(const std::string &s)
  {
    std::string out = "'";
    for (char c : s)
    {
      if (c == '\'') out += "'\\''";
      else out += c;
    }
    out += "'";
    return out;
  }

  void onSave(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
              std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    ray_mapper_->syncVoxels();
    const int rc = ohm::save(save_path_default_, *occ_map_);
    if (rc != 0)
    {
      resp->success = false;
      resp->message = "ohm::save failed with code " + std::to_string(rc);
      RCLCPP_ERROR(get_logger(), "%s (path=%s)", resp->message.c_str(),
                   save_path_default_.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Saved map to %s", save_path_default_.c_str());

    // Chain ohm2ply so a single /ohm/save call produces both the .ohm and
    // a visualisation-ready .ply. ohm2ply lives in /usr/local/bin in this
    // image (OHM's own CLI tools, installed by the Dockerfile).
    const std::string ply_path = derivePlyPath(save_path_default_);
    const std::string cmd = "ohm2ply --voxel-mode voxel "
        + shellQuote(save_path_default_) + " " + shellQuote(ply_path)
        + " >/dev/null 2>&1";
    const int ply_rc = std::system(cmd.c_str());
    if (ply_rc == 0)
    {
      resp->success = true;
      resp->message = "Saved " + save_path_default_ + " and " + ply_path;
      RCLCPP_INFO(get_logger(), "Exported %s", ply_path.c_str());
    }
    else
    {
      // .ohm is safe on disk — report partial success.
      resp->success = true;
      resp->message = "Saved " + save_path_default_
          + " (ohm2ply failed rc=" + std::to_string(ply_rc)
          + "; expected .ply path " + ply_path + ")";
      RCLCPP_WARN(get_logger(), "ohm2ply exited with code %d for %s",
                  ply_rc, ply_path.c_str());
    }
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OhmLiveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
