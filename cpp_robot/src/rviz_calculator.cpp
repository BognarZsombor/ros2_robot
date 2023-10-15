#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;
using std::placeholders::_1;


class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0), sending_dest(false)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

        this->dest_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, std::bind(&MinimalPublisher::dest_sub_callback, this, _1));

        this->location_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, std::bind(&MinimalPublisher::loc_sub_callback, this, _1));
    }

private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        auto msg_linear = geometry_msgs::msg::Vector3();
        auto msg_angular = geometry_msgs::msg::Vector3();

        if (this->sending_dest) {
            msg_linear.set__x(0.3);
        } else {
            msg_linear.set__x(0.0);
        }
        msg_linear.set__y(0.0);
        msg_linear.set__z(0.0);

        msg_angular.set__x(0.0);
        msg_angular.set__y(0.0);
        float thetat = atan2f(this->yt - this->yr, this->xt - this->xr);
        msg_angular.set__z(this->kp * (thetat - this->theta));

        msg.linear = msg_linear;
        msg.angular = msg_angular;

        if (abs(this->xt - this->xr) > 0.5 || abs(this->yt - this->yr) > 0.5) {
            this->publisher_->publish(msg);
        }
    }

    void dest_sub_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        this->xt = msg->pose.position.x;
        this->yt = msg->pose.position.y;
        if (this->xt != 0.0) {
            this->sending_dest = true;
        }
    }

    void loc_sub_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        this->theta = yaw;
        this->xr = msg->pose.position.x;
        this->yr = msg->pose.position.y;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dest_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr location_subscription_;
    size_t count_;

    float kp, xt, yt, xr, yr, theta;
    bool sending_dest;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}