#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using std::placeholders::_1;


class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0)
    {
        this->lasttime = this->get_clock()->now().nanoseconds();
        this->subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "topic", 10, std::bind(&MinimalPublisher::listener_callback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("display", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void multiplyBu(float firstMatrix[][2], float secondMatrix[][1], float mult[][1], int rowFirst, int columnFirst, int rowSecond, int columnSecond)
    {
        int i, j, k;

        // Initializing elements of matrix mult to 0.
        for(i = 0; i < rowFirst; ++i)
        {
            for(j = 0; j < columnSecond; ++j)
            {
                mult[i][j] = 0;
            }
        }

        // Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
        for(i = 0; i < rowFirst; ++i)
        {
            for(j = 0; j < columnSecond; ++j)
            {
                for(k=0; k<columnFirst; ++k)
                {
                    mult[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }
    }

    void multiplyAx(float firstMatrix[][3], float secondMatrix[][1], float mult[][1], int rowFirst, int columnFirst, int rowSecond, int columnSecond)
    {
        int i, j, k;

        // Initializing elements of matrix mult to 0.
        for(i = 0; i < rowFirst; ++i)
        {
            for(j = 0; j < columnSecond; ++j)
            {
                mult[i][j] = 0;
            }
        }

        // Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
        for(i = 0; i < rowFirst; ++i)
        {
            for(j = 0; j < columnSecond; ++j)
            {
                for(k=0; k<columnFirst; ++k)
                {
                    mult[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }
    }

    void addMatrices(float a[3][1], float b[3][1], float sum[3][1], int r, int c)
    {
        int i, j;

        for(i = 0; i < r; ++i)
            for(j = 0; j < c; ++j)
                sum[i][j] = a[i][j] + b[i][j];
        }

    void timer_callback()
    {
        this->time = this->get_clock()->now().nanoseconds();
        float dt = this->time - this->lasttime;

        float A[3][3] = {{1.0, 0.0, 0.0},
                         {0.0, 1.0, 0.0},
                         {0.0, 0.0, 1.0}};

        float xt[3][1] = {{this->x},
                          {this->y},
                          {this->theta}};

        float B[3][2] = {{cosf(this->theta) * dt, 0.0},
                         {sinf(this->theta) * dt, 0.0},
                         {0.0, dt}};

        float ut[2][1] = {{this->v},
                          {this->w}};

        float Ax[3][1];
        float Bu[3][1];
        this->multiplyAx(A, xt, Ax, 3, 3, 3, 1);
        this->multiplyBu(B, ut, Bu, 3, 3, 3, 1);

        this->lasttime = this->time;

        float result[3][1];
        this->addMatrices(Ax, Bu, result, 3, 1);

        auto msg = geometry_msgs::msg::PoseStamped();
        auto msg_point = geometry_msgs::msg::Point();
        auto msg_quat = geometry_msgs::msg::Quaternion();

        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, this->theta);
        quat = quat.normalize();

        msg_quat.set__x(quat.getX());
        msg_quat.set__y(quat.getY());
        msg_quat.set__z(quat.getZ());
        msg_quat.set__w(quat.getW());

        msg_point.set__x(this->x);
        msg_point.set__y(this->y);
        msg_point.set__z(0.0);

        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.pose.position = msg_point;
        msg.pose.orientation = msg_quat;

        RCLCPP_INFO(this->get_logger(), "I heard: '%lf'", msg.pose.position.x);

        publisher_->publish(msg);
    }

    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        this->v = msg->linear.x;
        this->w = msg->angular.z;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    size_t count_;

    float v, w, x, y, theta, time, lasttime;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}