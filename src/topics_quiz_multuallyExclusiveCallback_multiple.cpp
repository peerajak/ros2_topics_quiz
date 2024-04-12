/*
Applying
    4.12   Multiple Mutually Exclusive Callback Groups
    (Multiple means multiple group)
to
    TOPICS_QUIZ
*/

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace std::chrono_literals;

class LaserTimer : public rclcpp::Node {
public:
  LaserTimer(float sleep_timer1) : Node("topics_quiz_node2") {

    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_2;

    subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&LaserTimer::laser_callback, this, std::placeholders::_1),
        options1);

    this->wait_time1 = sleep_timer1;

    timer1_ = this->create_wall_timer(
        500ms, std::bind(&LaserTimer::timer1_callback, this), callback_group_1);

    publisher1_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void timer1_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback Start");
    sleep(this->wait_time1);
    this->move_robot(ling);
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback End");
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    int rangesize = 719;
    int midrange = rangesize / 2, left_onethird = rangesize / 3,
        right_onethird = rangesize * 2 / 3;
    if (msg->ranges[midrange] <= 1 || msg->ranges[left_onethird] <= 1 ||
        msg->ranges[right_onethird] <= 1) {
      RCLCPP_INFO(this->get_logger(), "collision ahead %d/%d:%f", midrange,
                  rangesize, msg->ranges[midrange]);
      ling.linear.x -= 0.001;
      ling.linear.y += 0.01;
      ling.angular.z = 1;
    } else {
      // RCLCPP_INFO(this->get_logger(), "%d/%d:%f", midrange, rangesize,
      //            msg->ranges[midrange]);
      ling.linear.x += 0.001;
      ling.angular.z = 0;
    }
  }
  void move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }
  geometry_msgs::msg::Twist ling;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  float wait_time1;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Instantiate the Node
  float sleep_time1 = 1.0;

  std::shared_ptr<LaserTimer> laser_timer_node =
      std::make_shared<LaserTimer>(sleep_time1);

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(laser_timer_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}