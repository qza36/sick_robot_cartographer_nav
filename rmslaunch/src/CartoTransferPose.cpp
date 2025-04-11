#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

class TfPosePublisher : public rclcpp::Node {
public:
  TfPosePublisher() : Node("tf_pose_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("pose_data", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TfPosePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    geometry_msgs::msg::Pose2D pos_now;
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
      return;
    }

    double x = transform_stamped.transform.translation.x;
    double y = transform_stamped.transform.translation.y;
    double z = transform_stamped.transform.translation.z;
    double qx = transform_stamped.transform.rotation.x;
    double qy = transform_stamped.transform.rotation.y;
    double qz = transform_stamped.transform.rotation.z;
    double qw = transform_stamped.transform.rotation.w;

    tf2::Quaternion q(qx, qy, qz, qw);
    double theta = tf2::getYaw(q);

    pos_now.x = x;
    pos_now.y = y;
    pos_now.theta = theta;

    pose_pub_->publish(pos_now);

    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f, theta: %f",
                x, y, z, qx, qy, qz, qw, theta);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfPosePublisher>());
  rclcpp::shutdown();
  return 0;
}