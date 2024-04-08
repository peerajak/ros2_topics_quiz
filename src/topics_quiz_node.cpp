#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class MoveRobot : public rclcpp::Node {
public:
  MoveRobot() : Node("move_robot") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // timer_ = this->create_wall_timer( 500ms,
    // std::bind(&MoveRobot::timer_callback, this));
  }
  void move_robot(geometry_msgs::msg::Twist &msg) { publisher_->publish(msg); }

private:
  /*void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.2;
    message.angular.z = 0.2;
    publisher_->publish(message);
  }*/
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

class LaserSubscriber : public rclcpp::Node {
public:
  LaserSubscriber() : Node("topics_quiz_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&LaserSubscriber::scan_callback, this, _1));
    robot_move = std::make_shared<MoveRobot>();
  }

private:
  std::shared_ptr<MoveRobot> robot_move;
  geometry_msgs::msg::Twist ling;
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

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
      RCLCPP_INFO(this->get_logger(), "%d/%d:%f", midrange, rangesize,
                  msg->ranges[midrange]);
      ling.linear.x += 0.001;
      ling.angular.z = 0;
    }
    robot_move->move_robot(ling);
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserSubscriber>());
  // rclcpp::spin(std::make_shared<MoveRobot>());
  rclcpp::shutdown();
  return 0;
}
