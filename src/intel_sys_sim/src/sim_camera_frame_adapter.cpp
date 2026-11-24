// Relabels the simulated D435's color, depth and camera_info with the optical frame.
//
// Gazebo stamps every topic of an rgbd_camera with one frame (gz_frame_id). Its point
// cloud is x forward, so that frame must be camera_color_frame (as Nav2 reads it). Images
// and camera_info follow the optical convention (z forward), which is what RTAB-Map and
// the real driver use, so they are republished here in camera_color_optical_frame.
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class SimCameraFrameAdapter : public rclcpp::Node
{
public:
  SimCameraFrameAdapter()
  : Node("sim_camera_frame_adapter")
  {
    frame_ = declare_parameter<std::string>("optical_frame", "camera_color_optical_frame");
    relay<sensor_msgs::msg::Image>("/camera/image_sim", "/camera/image_raw", image_pub_, image_sub_);
    relay<sensor_msgs::msg::Image>("/camera/depth/image_sim", "/camera/depth/image_raw", depth_pub_, depth_sub_);
    relay<sensor_msgs::msg::CameraInfo>("/camera/camera_info_sim", "/camera/camera_info", info_pub_, info_sub_);
  }

private:
  template<typename T>
  void relay(
    const std::string & in, const std::string & out,
    typename rclcpp::Publisher<T>::SharedPtr & pub,
    typename rclcpp::Subscription<T>::SharedPtr & sub)
  {
    // Reliable like the real driver, so RTAB-Map and Nav2 match it.
    pub = create_publisher<T>(out, rclcpp::QoS(5));
    auto * p = &pub;
    sub = create_subscription<T>(
      in, rclcpp::SensorDataQoS(),
      [this, p](typename T::UniquePtr msg) {
        msg->header.frame_id = frame_;
        (*p)->publish(std::move(msg));
      });
  }

  std::string frame_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_, depth_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_, depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimCameraFrameAdapter>());
  rclcpp::shutdown();
  return 0;
}
