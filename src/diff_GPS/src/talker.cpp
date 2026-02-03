#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Talker : public rclcpp::Node
{
public:
    Talker() : Node("talker")
    {
        // Declare parameters with default values
        this->declare_parameter<double>("v", 10.0);
        this->declare_parameter<double>("d", 1.0);

        // Read parameter values
        speed_ = this->get_parameter("v").as_double();
        steering_angle_ = this->get_parameter("d").as_double();

        // Create publisher
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    }

    void publish()
    {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        auto drive_msg = ackermann_msgs::msg::AckermannDrive();

        drive_msg.speed = speed_;
        drive_msg.steering_angle = steering_angle_;
        msg.drive = drive_msg;

        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    double speed_;
    double steering_angle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Talker>();

    while (rclcpp::ok()) {
        node->publish();   // publish as fast as possible
        rclcpp::spin_some(node); // optional: process callbacks (not needed here if no subscriptions)
    }

    rclcpp::shutdown();
    return 0;
}
