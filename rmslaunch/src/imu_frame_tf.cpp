#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class ImuFrameTransformer : public rclcpp::Node
{
public:
    ImuFrameTransformer() : Node("imu_frame_transformer")
    {
        auto qos = rclcpp::QoS(10)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", qos);
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sick_scansegment_xd/imu", 
            qos,
            std::bind(&ImuFrameTransformer::imuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IMU Frame Transformer initialized");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        try 
        {
            auto transformed_msg = std::make_unique<sensor_msgs::msg::Imu>();
            
            // 使用节点的当前时间作为时间戳
            rclcpp::Time current_stamp = this->now();
            
            // 设置时间戳和frame_id
            transformed_msg->header.stamp = current_stamp;
            transformed_msg->header.frame_id = "base_link";
            
            // 复制IMU数据
            transformed_msg->orientation = msg->orientation;
            transformed_msg->orientation_covariance = msg->orientation_covariance;
            transformed_msg->angular_velocity = msg->angular_velocity;
            transformed_msg->angular_velocity_covariance = msg->angular_velocity_covariance;
            transformed_msg->linear_acceleration = msg->linear_acceleration;
            transformed_msg->linear_acceleration_covariance = msg->linear_acceleration_covariance;
            
            publisher_->publish(std::move(transformed_msg));

            // 正确的方式访问时间戳
            RCLCPP_DEBUG(this->get_logger(), 
                "Published IMU message with time: %ld.%ld",
                current_stamp.seconds(),
                current_stamp.nanoseconds() % 1000000000L);
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "Error transforming IMU message: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuFrameTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}