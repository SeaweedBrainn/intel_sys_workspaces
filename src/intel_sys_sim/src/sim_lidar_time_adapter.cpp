#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>



class SimLidarTimeAdapter : public rclcpp::Node {
public:
  SimLidarTimeAdapter() : Node("sim_lidar_time_adapter") {
    this->declare_parameter<std::string>("input_topic", "/lidar/points_sim");
    this->declare_parameter<std::string>("output_topic", "/lidar/points_raw");
    this->declare_parameter<double>("scan_period", 0.1);  // 10 Hz

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    scan_period_ = static_cast<float>(this->get_parameter("scan_period").as_double());

    rclcpp::QoS sub_qos(10);
    sub_qos.best_effort();
    sub_qos.durability_volatile();

    rclcpp::QoS pub_qos(10);
    pub_qos.reliable();
    pub_qos.durability_volatile();

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, pub_qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, sub_qos,
        std::bind(&SimLidarTimeAdapter::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Sim LiDAR Time Adapter started: %s -> %s (period: %.2fs)",
                input_topic.c_str(), output_topic.c_str(), scan_period_);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Determine optional fields
    bool has_intensity = false;
    bool has_in_ring = false;
    for (const auto &f : msg->fields) {
      if (f.name == "intensity") has_intensity = true;
      if (f.name == "ring") has_in_ring = true;
    }

    sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");

    struct ProcessedPoint {
      float x, y, z, intensity, time;
      uint16_t ring;
    };
    std::vector<ProcessedPoint> valid_points;
    valid_points.reserve(msg->width * msg->height);

    const float two_pi = 2.0f * static_cast<float>(M_PI);
    const uint32_t HORIZONTAL_SAMPLES = 360;
    const uint32_t VERTICAL_SAMPLES = 18;
    uint32_t point_idx = 0;

    if (has_in_ring) {
      sensor_msgs::PointCloud2ConstIterator<uint16_t> in_ring(*msg, "ring");
      if (has_intensity) {
        sensor_msgs::PointCloud2ConstIterator<float> in_intensity(*msg, "intensity");
        for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z, ++in_ring, ++in_intensity) {
          float x = *in_x;
          float y = *in_y;
          float z = *in_z;
          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
          }
          float azimuth = std::atan2(y, x);
          float norm_angle = (azimuth + static_cast<float>(M_PI)) / two_pi;
          float t = norm_angle * scan_period_;
          valid_points.push_back({x, y, z, *in_intensity, t, *in_ring});
        }
      } else {
        for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z, ++in_ring) {
          float x = *in_x;
          float y = *in_y;
          float z = *in_z;
          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
          }
          float azimuth = std::atan2(y, x);
          float norm_angle = (azimuth + static_cast<float>(M_PI)) / two_pi;
          float t = norm_angle * scan_period_;
          valid_points.push_back({x, y, z, 100.0f, t, *in_ring});
        }
      }
    } else {
      if (has_intensity) {
        sensor_msgs::PointCloud2ConstIterator<float> in_intensity(*msg, "intensity");
        for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z, ++point_idx, ++in_intensity) {
          float x = *in_x;
          float y = *in_y;
          float z = *in_z;
          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
          }
          float azimuth = std::atan2(y, x);
          float norm_angle = (azimuth + static_cast<float>(M_PI)) / two_pi;
          float t = norm_angle * scan_period_;
          uint16_t ring = static_cast<uint16_t>((point_idx / HORIZONTAL_SAMPLES) % VERTICAL_SAMPLES);
          valid_points.push_back({x, y, z, *in_intensity, t, ring});
        }
      } else {
        for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z, ++point_idx) {
          float x = *in_x;
          float y = *in_y;
          float z = *in_z;
          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
          }
          float azimuth = std::atan2(y, x);
          float norm_angle = (azimuth + static_cast<float>(M_PI)) / two_pi;
          float t = norm_angle * scan_period_;
          uint16_t ring = static_cast<uint16_t>((point_idx / HORIZONTAL_SAMPLES) % VERTICAL_SAMPLES);
          valid_points.push_back({x, y, z, 100.0f, t, ring});
        }
      }
    }

    auto out_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out_msg->header = msg->header;
    out_msg->height = 1;
    out_msg->width = static_cast<uint32_t>(valid_points.size());
    out_msg->is_dense = true;
    out_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(*out_msg);
    modifier.setPointCloud2Fields(6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16
    );
    modifier.resize(valid_points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(*out_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(*out_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(*out_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(*out_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<float> out_time(*out_msg, "time");
    sensor_msgs::PointCloud2Iterator<uint16_t> out_ring(*out_msg, "ring");

    for (const auto &p : valid_points) {
      *out_x = p.x;
      *out_y = p.y;
      *out_z = p.z;
      *out_intensity = p.intensity;
      *out_time = p.time;
      *out_ring = p.ring;

      ++out_x;
      ++out_y;
      ++out_z;
      ++out_intensity;
      ++out_time;
      ++out_ring;
    }

    pub_->publish(std::move(out_msg));
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  float scan_period_{0.1f};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimLidarTimeAdapter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
