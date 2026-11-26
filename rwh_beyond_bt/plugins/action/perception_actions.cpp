// Copyright 2026 rwh_beyond contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rwh_beyond_bt/plugins/action/perception_actions.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "rwh_beyond_bt/diagnostics.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace rwh_beyond_bt
{
namespace
{
int64_t stamp(const std_msgs::msg::Header & h)
{return rclcpp::Time(h.stamp).nanoseconds();}

template<typename T>
T required(const BT::TreeNode & node, const std::string & port)
{
  auto value = node.getInput<T>(port);
  if (!value) {throw std::invalid_argument("Missing " + port + ": " + value.error());}
  return value.value();
}

template<typename T>
T setting(const rclcpp::Node::SharedPtr & node, const std::string & key, const T & fallback)
{
  const auto name = "bt_perception." + key;
  if (!node->has_parameter(name)) {node->declare_parameter<T>(name, fallback);}
  return node->get_parameter(name).get_value<T>();
}

BT::NodeStatus failed(
  const BT::TreeNode & node, const std::string & source,
  const std::string & message, int32_t code = ErrorCode::INVALID_INPUT,
  const std::string & name = "INVALID_INPUT", const std::string & domain = "bt")
{
  report_failure(node.config().blackboard, code, name, source, message, domain);
  return BT::NodeStatus::FAILURE;
}

template<class Result>
BT::NodeStatus model_failure(
  const BT::TreeNode & node, const std::shared_ptr<Result> & result, const std::string & source)
{
  if (!result || result->error_code == 0 || result->error_domain.empty() ||
    result->error_source.empty())
  {
    return failed(node, source, "Model action returned no structured failure",
          ErrorCode::INVALID_RESPONSE,
      "INVALID_RESPONSE", "transport");
  }
  return failed(node, result->error_source, result->error_msg, result->error_code,
    result->error_name, result->error_domain);
}

void validate_image(const sensor_msgs::msg::Image & image, size_t pixel_size)
{
  if (!image.width || !image.height || uint64_t(image.width) * image.height > 2073600 ||
    image.step < uint64_t(image.width) * pixel_size ||
    image.data.size() != uint64_t(image.step) * image.height ||
    image.data.size() > 32 * 1024 * 1024)
  {throw std::invalid_argument("Invalid image dimensions, stride or data size");}
}
}  // namespace

sensor_msgs::msg::PointCloud2 project_object_cloud(
  const RgbdObservation & observation, const sensor_msgs::msg::Image & mask)
{
  const auto & depth = observation.depth;
  const auto & info = observation.info;
  const bool millimeters = depth.encoding == "16UC1";
  if (!millimeters && depth.encoding != "32FC1") {
    throw std::invalid_argument("Depth encoding must be 16UC1 or 32FC1");
  }
  if (mask.encoding != "mono8" && mask.encoding != "8UC1") {
    throw std::invalid_argument("Mask encoding must be mono8 or 8UC1");
  }
  validate_image(depth, millimeters ? 2 : 4);
  validate_image(mask, 1);
  if (depth.header.frame_id.empty() || mask.header.frame_id != depth.header.frame_id ||
    info.header.frame_id != depth.header.frame_id ||
    mask.width != depth.width || mask.height != depth.height ||
    info.width != depth.width || info.height != depth.height ||
    std::abs(stamp(mask.header) - stamp(depth.header)) > 100000000 ||
    (stamp(info.header) && std::abs(stamp(info.header) - stamp(depth.header)) > 100000000))
  {
    throw std::invalid_argument(
        "Mask, depth and calibration must describe one registered exposure");
  }
  if (info.binning_x > 1 || info.binning_y > 1 || info.roi.x_offset || info.roi.y_offset ||
    !std::all_of(info.k.begin(), info.k.end(), [](double v) {return std::isfinite(v);}) ||
    info.k[0] <= 0 || info.k[4] <= 0 || std::abs(info.k[6]) > 1e-8 ||
    std::abs(info.k[7]) > 1e-8 || std::abs(info.k[8] - 1) > 1e-8 ||
    std::abs(info.k[3]) > 1e-8 ||
    !std::all_of(info.d.begin(), info.d.end(), [](double v) {return std::isfinite(v);}))
  {throw std::invalid_argument("Invalid full-resolution pinhole calibration");}
  const bool distorted = std::any_of(info.d.begin(), info.d.end(),
      [](double v) {return v != 0;});
  if (distorted && (info.distortion_model != "plumb_bob" ||
    (info.d.size() != 5 && info.d.size() != 8)))
  {throw std::invalid_argument("Unsupported nonzero camera distortion");}
  std::vector<std::array<float, 3>> xyz;
  const uint16_t endian = 1;
  const bool swap = static_cast<bool>(depth.is_bigendian) !=
    (*reinterpret_cast<const uint8_t *>(&endian) == 0);
  for (uint32_t row = 0; row < depth.height; ++row) {
    for (uint32_t col = 0; col < depth.width; ++col) {
      if (!mask.data[size_t(row) * mask.step + col]) {continue;}
      std::array<uint8_t, 4> bytes{};
      const size_t size = millimeters ? 2 : 4;
      std::copy_n(depth.data.data() + size_t(row) * depth.step + col * size, size, bytes.begin());
      if (swap) {std::reverse(bytes.begin(), bytes.begin() + size);}
      float z;
      if (millimeters) {
        uint16_t value;
        std::memcpy(&value, bytes.data(), 2);
        z = value * 0.001f;
      } else {std::memcpy(&z, bytes.data(), 4);}
      if (!std::isfinite(z) || z <= 0) {continue;}
      const double tx = (col - info.k[2]) / info.k[0];
      const double ty = (row - info.k[5]) / info.k[4];
      double x = tx, y = ty;
      if (distorted) {
        const auto & d = info.d;
        for (int j = 0; j < 8; ++j) {
          const double r2 = x * x + y * y;
          const double numerator = 1 + d[0] * r2 + d[1] * r2 * r2 + d[4] * r2 * r2 * r2;
          const double denominator = d.size() == 8 ?
            1 + d[5] * r2 + d[6] * r2 * r2 + d[7] * r2 * r2 * r2 : 1;
          const double radial = numerator / denominator;
          const double dx = 2 * d[2] * x * y + d[3] * (r2 + 2 * x * x);
          const double dy = d[2] * (r2 + 2 * y * y) + 2 * d[3] * x * y;
          const double nx = x + tx - (x * radial + dx);
          const double ny = y + ty - (y * radial + dy);
          x = nx; y = ny;
        }
      }
      const std::array<float, 3> point{static_cast<float>(x * z), static_cast<float>(y * z), z};
      if (!std::all_of(point.begin(), point.end(), [](float v) {return std::isfinite(v);})) {
        throw std::invalid_argument("Nonfinite projected XYZ");
      }
      xyz.push_back(point);
    }
  }
  if (xyz.size() < 100) {throw std::invalid_argument("At least 100 target depth points required");}
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = depth.header;
  cloud.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  const size_t count = std::min<size_t>(xyz.size(), 200000);
  modifier.resize(count);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (size_t i = 0; i < count; ++i, ++x, ++y, ++z) {
    const auto & p = xyz[i * (xyz.size() - 1) / (count - 1)];
    *x = p[0]; *y = p[1]; *z = p[2];
  }
  return cloud;
}

struct CaptureRgbdAction::State
{
  rclcpp::Node::SharedPtr node;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub, depth_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub;
  std::map<int64_t, RgbdObservation> frames;
  sensor_msgs::msg::JointState joints;
  std::chrono::steady_clock::time_point deadline, joints_received;
  int64_t after = 0;
};

CaptureRgbdAction::CaptureRgbdAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), state_(std::make_unique<State>())
{
  auto & s = *state_;
  s.node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  s.group = s.node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  s.executor.add_callback_group(s.group, s.node->get_node_base_interface());
  rclcpp::SubscriptionOptions options;
  options.callback_group = s.group;
  s.rgb_sub = s.node->create_subscription<sensor_msgs::msg::Image>(
    setting<std::string>(s.node, "rgb_topic", "camera/color/image_raw"), rclcpp::SensorDataQoS(),
    [&s](sensor_msgs::msg::Image::ConstSharedPtr m) {
      s.frames[stamp(m->header)].rgb = *m;
      }, options);
  s.depth_sub = s.node->create_subscription<sensor_msgs::msg::Image>(
    setting<std::string>(s.node, "depth_topic", "camera/depth/image_rect_raw"),
      rclcpp::SensorDataQoS(),
    [&s](sensor_msgs::msg::Image::ConstSharedPtr m) {
      s.frames[stamp(m->header)].depth = *m;
      }, options);
  s.info_sub = s.node->create_subscription<sensor_msgs::msg::CameraInfo>(
    setting<std::string>(s.node, "info_topic", "camera/depth/camera_info"), rclcpp::SensorDataQoS(),
    [&s](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
      s.frames[stamp(m->header)].info = *m;
      }, options);
  s.joints_sub = s.node->create_subscription<sensor_msgs::msg::JointState>(
    "feedback/joint_states", rclcpp::SensorDataQoS(),
    [&s](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      s.joints = *m; s.joints_received = std::chrono::steady_clock::now();
    }, options);
}
CaptureRgbdAction::~CaptureRgbdAction() = default;
BT::PortsList CaptureRgbdAction::providedPorts()
{
  return {BT::InputPort<double>("timeout", 10.0, "Fresh exposure wait in seconds"),
    BT::OutputPort<Observation>("capture"),
    BT::OutputPort<std::string>("observation_id")};
}
BT::NodeStatus CaptureRgbdAction::onStart()
{
  begin_stage(config().blackboard, "capture_rgbd", name());
  auto & s = *state_;
  s.frames.clear();
  s.after = s.node->now().nanoseconds();
  const auto timeout = required<double>(*this, "timeout");
  if (!std::isfinite(timeout) || timeout <= 0 || timeout > 30) {
    return failed(*this, "bt/capture_rgbd", "Timeout must be in (0,30] seconds");
  }
  s.deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(
      timeout));
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus CaptureRgbdAction::onRunning()
{
  auto & s = *state_;
  s.executor.spin_some();
  while (s.frames.size() > 8) {s.frames.erase(s.frames.begin());}
  for (auto it = s.frames.rbegin(); it != s.frames.rend(); ++it) {
    auto & observation = it->second;
    if (it->first <= s.after || !observation.rgb.width || !observation.depth.width ||
      !observation.info.width || s.joints.name.empty() ||
      std::chrono::steady_clock::now() - s.joints_received > std::chrono::seconds(1)) {continue;}
    if (observation.rgb.width != observation.depth.width ||
      observation.rgb.height != observation.depth.height ||
      observation.rgb.header.frame_id != observation.depth.header.frame_id ||
      observation.info.width != observation.depth.width ||
      observation.info.height != observation.depth.height ||
      observation.info.header.frame_id != observation.depth.header.frame_id)
    {return failed(*this, "bt/capture_rgbd", "RGB-D must be registered to the same optical frame");}
    bool complete_joints = s.joints.name.size() == s.joints.position.size();
    for (int i = 1; i <= 6 && complete_joints; ++i) {
      const auto joint = std::find(s.joints.name.begin(), s.joints.name.end(),
          "joint" + std::to_string(i));
      complete_joints = joint != s.joints.name.end() &&
        std::isfinite(s.joints.position[std::distance(s.joints.name.begin(), joint)]);
    }
    if (!complete_joints) {continue;}
    observation.joints = s.joints;
    observation.id = std::to_string(it->first);
    setOutput("capture", Observation(std::make_shared<RgbdObservation>(observation)));
    setOutput("observation_id", observation.id);
    return BT::NodeStatus::SUCCESS;
  }
  if (std::chrono::steady_clock::now() > s.deadline) {
    return failed(*this, "bt/capture_rgbd",
        "Fresh synchronized RGB-D/calibration/joints unavailable", ErrorCode::TIMEOUT, "TIMEOUT");
  }
  return BT::NodeStatus::RUNNING;
}
void CaptureRgbdAction::onHalted() {state_->frames.clear();}

sensor_msgs::msg::PointCloud2 largest_object_cluster(sensor_msgs::msg::PointCloud2 cloud)
{
  constexpr double tolerance_m = 0.01;
  constexpr int min_points = 100;
  auto points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  points->reserve(cloud.width);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    points->push_back(pcl::PointXYZ(*x, *y, *z));
  }
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
  extraction.setInputCloud(points);
  extraction.setSearchMethod(pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>(false));
  extraction.setClusterTolerance(tolerance_m);
  extraction.setMinClusterSize(min_points);
  std::vector<pcl::PointIndices> clusters;
  extraction.extract(clusters);
  if (clusters.empty()) {
    throw std::invalid_argument("No connected object cluster with at least 100 points at 1 cm");
  }
  const auto largest = std::max_element(clusters.begin(), clusters.end(),
      [](const auto & a, const auto & b) {
        if (a.indices.size() != b.indices.size()) {return a.indices.size() < b.indices.size();}
        return a.indices.front() > b.indices.front();
    });
  const auto & indices = largest->indices;  // PCL returns each cluster in input index order.
  for (size_t i = 0; i < indices.size(); ++i) {
    const auto source = static_cast<size_t>(indices[i]);
    if (source != i) {
      std::copy_n(cloud.data.data() + source * cloud.point_step, cloud.point_step,
        cloud.data.data() + i * cloud.point_step);
    }
  }
  sensor_msgs::PointCloud2Modifier(cloud).resize(indices.size());
  return cloud;
}

BuildObjectCloudAction::BuildObjectCloudAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "grasp/object_cloud", rclcpp::QoS(1).reliable().transient_local());
}
BT::PortsList BuildObjectCloudAction::providedPorts()
{
  return {BT::InputPort<Observation>("capture"), BT::InputPort<sensor_msgs::msg::Image>("mask"),
    BT::OutputPort<sensor_msgs::msg::PointCloud2>("cloud")};
}
BT::NodeStatus BuildObjectCloudAction::tick()
{
  begin_stage(config().blackboard, "build_object_cloud", name());
  try {
    const auto capture = required<Observation>(*this, "capture");
    if (!capture) {throw std::invalid_argument("Empty RGB-D capture");}
    const auto cloud = largest_object_cluster(project_object_cloud(*capture,
        required<sensor_msgs::msg::Image>(*this, "mask")));
    setOutput("cloud", cloud);
    publisher_->publish(cloud);
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & e) {
    return failed(*this, "bt/build_object_cloud", e.what());
  }
}

SegmentObjectAction::SegmentObjectAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BtActionNode(name, "segment_image", config) {}
BT::PortsList SegmentObjectAction::providedPorts()
{
  return providedBasicPorts({BT::InputPort<Observation>("capture"),
             BT::InputPort<std::string>("target_text", "", "Optional target description"),
             BT::InputPort<std::string>("exchange_view", "front", "File exchange view"),
             BT::OutputPort<sensor_msgs::msg::Image>("mask")});
}
BT::NodeStatus SegmentObjectAction::tick()
{
  try {
    const auto status = BtActionNode::tick();
    if (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() > deadline_) {
      failed(*this, "/segment_image", "Bridge result timed out; cancellation requested",
        ErrorCode::TIMEOUT, "TIMEOUT", "transport");
      BtActionNode::halt();
      return BT::NodeStatus::FAILURE;
    }
    if (status == BT::NodeStatus::FAILURE &&
      execution_diagnostic(config().blackboard).error_code == 0)
    {
      return failed(*this, "/segment_image",
          "Bridge goal request failed, timed out or was rejected",
        ErrorCode::ACTION_REQUEST_FAILED, "ACTION_REQUEST_FAILED", "transport");
    }
    return status;
  } catch (const std::exception & error) {
    report_exception(config().blackboard, error, "/segment_image");
    return BT::NodeStatus::FAILURE;
  }
}
void SegmentObjectAction::on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback)
{
  if (feedback) {backend_stage(config().blackboard, feedback->stage);}
}
BT::NodeStatus SegmentObjectAction::on_cancelled()
{
  if (result_.result && result_.result->error_code != 0) {
    return model_failure(*this, result_.result, "/segment_image");
  }
  return failed(*this, "/segment_image", "Model action canceled", ErrorCode::CANCELED, "CANCELED",
    "transport");
}
void SegmentObjectAction::on_tick()
{
  begin_stage(config().blackboard, "segment_object", name());
  deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(70);
  const auto capture = required<Observation>(*this, "capture");
  if (!capture) {throw std::invalid_argument("Empty RGB-D capture");}
  goal_ = Action::Goal();
  if (config().blackboard->rootBlackboard()->get(
      "exchange_request_id", goal_.exchange_request_id) && !goal_.exchange_request_id.empty())
  {
    goal_.exchange_view = required<std::string>(*this, "exchange_view");
  }
  goal_.rgb_image = capture->rgb;
  goal_.target_text = required<std::string>(*this, "target_text");
  std::string requested;
  if (config().blackboard->rootBlackboard()->get("target_text", requested) && !requested.empty()) {
    goal_.target_text = requested;
  }
  if (goal_.target_text.empty()) {
    goal_.target_text = setting<std::string>(node_, "target_text_override", "");
    if (goal_.target_text.empty()) {
      goal_.target_text = setting<std::string>(node_, "target_text", "yellow block");
    }
  }
  goal_.selection_policy = setting<std::string>(node_, "selection_policy", "highest_score");
  const auto box = setting<std::vector<double>>(node_, "target_box", {0., 0., 0., 0.});
  const auto point = setting<std::vector<int64_t>>(node_, "selection_point", {-1, -1});
  if (box.size() != 4 || point.size() != 2) {
    throw std::invalid_argument("Invalid SAM prompt dimensions");
  }
  std::copy(box.begin(), box.end(), goal_.target_box.begin());
  goal_.use_selection_point = point[0] >= 0 && point[1] >= 0;
  if (goal_.use_selection_point) {goal_.selection_u = point[0]; goal_.selection_v = point[1];}
}
BT::NodeStatus SegmentObjectAction::on_success()
{
  if (!result_.result || !result_.result->success || result_.result->error_code != 0) {
    return on_aborted();
  }
  if (result_.result->mock && !setting<bool>(node_, "allow_mock_segmentation", false)) {
    return failed(*this, "/segment_image", "Mock segmentation is disabled",
      ErrorCode::MOCK_NOT_ALLOWED, "MOCK_NOT_ALLOWED");
  }
  setOutput("mask", result_.result->object_mask);
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus SegmentObjectAction::on_aborted()
{
  return model_failure(*this, result_.result, "/segment_image");
}

InferGraspsAction::InferGraspsAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BtActionNode(name, "infer_grasps", config) {}
BT::PortsList InferGraspsAction::providedPorts()
{
  return providedBasicPorts({BT::InputPort<sensor_msgs::msg::PointCloud2>("cloud"),
             BT::InputPort<std::string>("exchange_view", "front", "File exchange view"),
             BT::OutputPort<std::vector<rwh_beyond_msgs::msg::GraspCandidate>>("candidates")});
}
BT::NodeStatus InferGraspsAction::tick()
{
  try {
    const auto status = BtActionNode::tick();
    if (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() > deadline_) {
      failed(*this, "/infer_grasps", "Bridge result timed out; cancellation requested",
        ErrorCode::TIMEOUT, "TIMEOUT", "transport");
      BtActionNode::halt();
      return BT::NodeStatus::FAILURE;
    }
    if (status == BT::NodeStatus::FAILURE &&
      execution_diagnostic(config().blackboard).error_code == 0)
    {
      return failed(*this, "/infer_grasps", "Bridge goal request failed, timed out or was rejected",
        ErrorCode::ACTION_REQUEST_FAILED, "ACTION_REQUEST_FAILED", "transport");
    }
    return status;
  } catch (const std::exception & error) {
    report_exception(config().blackboard, error, "/infer_grasps");
    return BT::NodeStatus::FAILURE;
  }
}
void InferGraspsAction::on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback)
{
  if (feedback) {backend_stage(config().blackboard, feedback->stage);}
}
BT::NodeStatus InferGraspsAction::on_cancelled()
{
  if (result_.result && result_.result->error_code != 0) {
    return model_failure(*this, result_.result, "/infer_grasps");
  }
  return failed(*this, "/infer_grasps", "Model action canceled", ErrorCode::CANCELED, "CANCELED",
    "transport");
}
void InferGraspsAction::on_tick()
{
  begin_stage(config().blackboard, "infer_grasps", name());
  deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(70);
  goal_ = Action::Goal();
  if (config().blackboard->rootBlackboard()->get(
      "exchange_request_id", goal_.exchange_request_id) && !goal_.exchange_request_id.empty())
  {
    goal_.exchange_view = required<std::string>(*this, "exchange_view");
  }
  goal_.input_mode = Action::Goal::POINT_CLOUD;
  goal_.object_cloud = required<sensor_msgs::msg::PointCloud2>(*this, "cloud");
  goal_.gripper_name = "piper_hand";
  goal_.num_grasps = setting<int64_t>(node_, "num_grasps", 1000);
  goal_.top_k = setting<int64_t>(node_, "top_k", 500);
}
BT::NodeStatus InferGraspsAction::on_success()
{
  if (!result_.result || !result_.result->success || result_.result->error_code != 0) {
    return on_aborted();
  }
  if (result_.result->mock) {
    return failed(*this, "/infer_grasps", "Mock grasps are disabled", ErrorCode::MOCK_NOT_ALLOWED,
      "MOCK_NOT_ALLOWED");
  }
  if (result_.result->candidates.empty()) {
    return failed(*this, "/infer_grasps", "Successful model action returned no candidates",
      ErrorCode::NO_CANDIDATES, "NO_CANDIDATES");
  }
  setOutput("candidates", result_.result->candidates);
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus InferGraspsAction::on_aborted()
{
  return model_failure(*this, result_.result, "/infer_grasps");
}
}  // namespace rwh_beyond_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rwh_beyond_bt::CaptureRgbdAction>("CaptureRgbd");
  factory.registerNodeType<rwh_beyond_bt::BuildObjectCloudAction>("BuildObjectCloud");
  factory.registerNodeType<rwh_beyond_bt::SegmentObjectAction>("SegmentObject");
  factory.registerNodeType<rwh_beyond_bt::InferGraspsAction>("InferGrasps");
}
