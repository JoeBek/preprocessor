#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// node declaration
class ZedImageSubscriber : public rclcpp::Node
{
public:

  // constructor
  ZedImageSubscriber() : Node("zed_image_subscriber")
  {
    // create topic subscription
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/zed/zed_node/left/image_rect_color", 10,
      std::bind(&ZedImageSubscriber::image_callback, this, std::placeholders::_1));
  }

private:
  // image 
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    try
    {
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::imshow("ZED Left Image", image);
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}

