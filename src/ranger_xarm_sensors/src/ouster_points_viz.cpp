#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class OusterPointsViz : public rclcpp::Node
{
public:
  OusterPointsViz()
  : Node("ouster_points_viz")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/ouster/points");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/ouster/points_viz");
    point_stride_ = std::max<int64_t>(1, this->declare_parameter<int64_t>("point_stride", 16));
    publish_every_nth_ =
      std::max<int64_t>(1, this->declare_parameter<int64_t>("publish_every_nth", 2));

    auto qos = rclcpp::SensorDataQoS();
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&OusterPointsViz::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Reducing %s -> %s: keep every %ldth point and publish every %ldth cloud "
      "(nominal payload reduction ~%ldx).",
      input_topic_.c_str(), output_topic_.c_str(),
      static_cast<long>(point_stride_), static_cast<long>(publish_every_nth_),
      static_cast<long>(point_stride_ * publish_every_nth_));
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++received_clouds_;
    if (((received_clouds_ - 1U) % static_cast<uint64_t>(publish_every_nth_)) != 0U) {
      return;
    }

    if (msg->width == 0U || msg->height == 0U || msg->point_step == 0U) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Input PointCloud2 is empty or has point_step=0; skipping.");
      return;
    }

    const uint64_t total_points =
      static_cast<uint64_t>(msg->width) * static_cast<uint64_t>(msg->height);
    const uint64_t capacity_points =
      (total_points + static_cast<uint64_t>(point_stride_) - 1U) /
      static_cast<uint64_t>(point_stride_);

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1U;
    out.fields = msg->fields;
    out.is_bigendian = msg->is_bigendian;
    out.point_step = msg->point_step;
    out.is_dense = msg->is_dense;
    out.data.resize(static_cast<size_t>(capacity_points) * msg->point_step);

    uint64_t copied = 0U;
    for (
      uint64_t src_index = 0U;
      src_index < total_points;
      src_index += static_cast<uint64_t>(point_stride_))
    {
      const uint64_t row = src_index / static_cast<uint64_t>(msg->width);
      const uint64_t col = src_index % static_cast<uint64_t>(msg->width);
      const uint64_t src_offset =
        row * static_cast<uint64_t>(msg->row_step) +
        col * static_cast<uint64_t>(msg->point_step);
      const uint64_t dst_offset = copied * static_cast<uint64_t>(msg->point_step);

      if (src_offset + msg->point_step > msg->data.size()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "PointCloud2 layout exceeds data buffer; publishing the valid sampled prefix.");
        break;
      }

      std::memcpy(
        out.data.data() + static_cast<size_t>(dst_offset),
        msg->data.data() + static_cast<size_t>(src_offset),
        msg->point_step);
      ++copied;
    }

    out.width = static_cast<uint32_t>(copied);
    out.row_step = out.width * out.point_step;
    out.data.resize(static_cast<size_t>(out.row_step));
    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  int64_t point_stride_{16};
  int64_t publish_every_nth_{2};
  uint64_t received_clouds_{0U};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OusterPointsViz>());
  rclcpp::shutdown();
  return 0;
}
