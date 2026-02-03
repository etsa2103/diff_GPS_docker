#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Relay : public rclcpp::Node
{
public:
    Relay() : Node("relay"), speed_(0.0), steering_angle_(0.0)
    {
        // Create subscriber
        subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "drive", 10, [this](const ackermann_msgs::msg::AckermannDriveStamped & msg) {this->topic_callback(msg);}
        );

        // Create publisher
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
    }

    void publish()
    {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        auto drive_msg = ackermann_msgs::msg::AckermannDrive();

        drive_msg.speed = speed_ * 3.0;
        drive_msg.steering_angle = steering_angle_ * 3.0;
        msg.drive = drive_msg;
        
        publisher_->publish(msg);
    }

private:
    void topic_callback(const ackermann_msgs::msg::AckermannDriveStamped & msg)
        {
        // Store incoming speed and steering
        speed_ = msg.drive.speed;
        steering_angle_ = msg.drive.steering_angle;

        // Immediately publish multiplied values
        this->publish();
        }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    double speed_;
    double steering_angle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Relay>());
  rclcpp::shutdown();
  return 0;
}
