#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>

// Patchwork++-ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include "GroundSegmentationServer.hpp"
#include "Utils.hpp"

namespace patchworkpp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

GroundSegmentationServer::GroundSegmentationServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("patchworkpp_node", options) {
  params_.sensor_height = declare_parameter<double>("sensor_height", params_.sensor_height);
  params_.num_iter      = declare_parameter<int>("num_iter", params_.num_iter);
  params_.num_lpr       = declare_parameter<int>("num_lpr", params_.num_lpr);
  params_.num_min_pts   = declare_parameter<int>("num_min_pts", params_.num_min_pts);
  params_.th_seeds      = declare_parameter<double>("th_seeds", params_.th_seeds);

  params_.th_dist    = declare_parameter<double>("th_dist", params_.th_dist);
  params_.th_seeds_v = declare_parameter<double>("th_seeds_v", params_.th_seeds_v);
  params_.th_dist_v  = declare_parameter<double>("th_dist_v", params_.th_dist_v);

  params_.max_range       = declare_parameter<double>("max_range", params_.max_range);
  params_.min_range       = declare_parameter<double>("min_range", params_.min_range);
  params_.uprightness_thr = declare_parameter<double>("uprightness_thr", params_.uprightness_thr);

  params_.verbose    = declare_parameter<bool>("verbose", params_.verbose);
  params_.enable_RNR = declare_parameter<bool>("enable_rnr", params_.enable_RNR);

  // Construct the main Patchwork++ node
  Patchworkpp_ = std::make_unique<patchwork::PatchWorkpp>(params_);

  // Initialize subscribers
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_topic",
      rclcpp::SensorDataQoS(),
      std::bind(&GroundSegmentationServer::EstimateGround, this, std::placeholders::_1));

  ground_publisher_    = create_publisher<sensor_msgs::msg::PointCloud2>("/patchworkpp/ground",
                                                                      rclcpp::SensorDataQoS());
  nonground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/patchworkpp/nonground",
                                                                         rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "Patchwork++ ROS 2 node initialized");
}

void GroundSegmentationServer::EstimateGround(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  const auto &cloud = patchworkpp_ros::utils::PointCloud2ToEigenMat(msg);

  if (cloud.cols() < 4 && params_.enable_RNR) {
    RCLCPP_WARN(
        this->get_logger(),
        "PointCloud2 message does not contain the intensity field, the RNR filter is disabled. ");
  }
  // Estimate ground
  Patchworkpp_->estimateGround(cloud);

  // Get ground and nonground
  Eigen::MatrixX3f ground    = Patchworkpp_->getGround();
  Eigen::MatrixX3f nonground = Patchworkpp_->getNonground();
  double time_taken          = Patchworkpp_->getTimeTaken();
  PublishClouds(ground, nonground, msg->header);
}

void GroundSegmentationServer::PublishClouds(const Eigen::MatrixX3f &est_ground,
                                             const Eigen::MatrixX3f &est_nonground,
                                             const std_msgs::msg::Header header_msg) {
  std_msgs::msg::Header header = header_msg;
  ground_publisher_->publish(
      std::move(patchworkpp_ros::utils::EigenMatToPointCloud2(est_ground, header)));
  nonground_publisher_->publish(
      std::move(patchworkpp_ros::utils::EigenMatToPointCloud2(est_nonground, header)));
}
}  // namespace patchworkpp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(patchworkpp_ros::GroundSegmentationServer)
