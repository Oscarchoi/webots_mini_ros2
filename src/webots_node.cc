#include <cmath>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

using namespace webots;

namespace {

template <typename T>
T get_or_default_parameter(rclcpp::Node* node,
                           const std::string& param_name,
                           const T& default_value) {
  if (!node->has_parameter(param_name)) {
    node->declare_parameter<T>(param_name, default_value);
  }
  return node->get_parameter(param_name).get_value<T>();
}

}  // namespace

class WebotsRos2Node : public rclcpp::Node {
 public:
  WebotsRos2Node()
      : Node("webots_ros2_node"), latest_rps_updated_(this->now()) {
    RCLCPP_INFO(this->get_logger(), "Starting Webots ROS2 Node...");

    time_step_ = get_or_default_parameter(this, "time_step", 200);
    wheelbase_ = get_or_default_parameter(this, "wheelbase", 0.09);
    wheel_radius_ = get_or_default_parameter(this, "wheel_radius", 0.025);
    cmd_vel_duration_sec_ =
        get_or_default_parameter(this, "cmd_vel_duration_sec", 1);
    cmd_vel_duration_usec_ =
        get_or_default_parameter(this, "cmd_vel_duration_usec", 0);

    auto command_topic = get_or_default_parameter(this, "command_topic",
                                                  std::string("/cmd_vel"));
    auto odom_topic = get_or_default_parameter(this, "odom_topic",
                                               std::string("/odom/wheel"));
    auto left_wheel_motor = get_or_default_parameter(
        this, "left_wheel_motor", std::string("left wheel motor"));
    auto right_wheel_motor = get_or_default_parameter(
        this, "right_wheel_motor", std::string("right wheel motor"));

    // Initialize Webots Object
    robot_ = new Robot();

    // Initialize Webots Device
    sensor_ = robot_->getDistanceSensor("ds0");
    if (sensor_) {
      sensor_->enable(time_step_);
      RCLCPP_INFO(this->get_logger(), "Found distance sensor: %s", "ds0");
    }

    motor_left_ = robot_->getMotor(left_wheel_motor);
    if (motor_left_) {
      motor_left_->setPosition(INFINITY);
      motor_left_->setVelocity(1.0);
      RCLCPP_INFO(this->get_logger(), "Found motor: '%s'",
                  left_wheel_motor.c_str());
    }
    position_sensor_left_ = motor_left_->getPositionSensor();
    if (position_sensor_left_) {
      position_sensor_left_->enable(time_step_);
      RCLCPP_INFO(this->get_logger(), "Found position sensor of motor: '%s'",
                  left_wheel_motor.c_str());
    }

    motor_right_ = robot_->getMotor(right_wheel_motor);
    if (motor_right_) {
      motor_right_->setPosition(INFINITY);
      motor_right_->setVelocity(1.0);
      RCLCPP_INFO(this->get_logger(), "Found motor: '%s'",
                  right_wheel_motor.c_str());
    }
    position_sensor_right_ = motor_right_->getPositionSensor();
    if (position_sensor_right_) {
      position_sensor_right_->enable(time_step_);
      RCLCPP_INFO(this->get_logger(), "Found position sensor of motor: '%s'",
                  right_wheel_motor.c_str());
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(time_step_),
                               std::bind(&WebotsRos2Node::update, this));
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        command_topic, 10,
        std::bind(&WebotsRos2Node::onCommand, this, std::placeholders::_1));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  }

  ~WebotsRos2Node() { delete robot_; }

 private:
  int time_step_;
  double wheelbase_;
  double wheel_radius_;
  int32_t cmd_vel_duration_sec_;
  uint32_t cmd_vel_duration_usec_;

  Robot* robot_;

  Motor* motor_left_;
  PositionSensor* position_sensor_left_;

  Motor* motor_right_;
  PositionSensor* position_sensor_right_;

  DistanceSensor* sensor_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;

  double target_rps_left_;
  double target_rps_right_;
  double wheel_velocity_left_ = 0.0;
  double wheel_velocity_right_ = 0.0;
  double wheel_position_left_ = 0.0;
  double wheel_position_right_ = 0.0;
  double wheel_position_left_prev_ = NAN;
  double wheel_position_right_prev_ = NAN;
  rclcpp::Time latest_rps_updated_;

  void onCommand(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double vx = msg->linear.x;
    double vth = msg->angular.z;
    target_rps_left_ = (vx - (vth * wheelbase_ / 2.0)) / wheel_radius_;
    target_rps_right_ = (vx + (vth * wheelbase_ / 2.0)) / wheel_radius_;
    latest_rps_updated_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Received command: vx=%f, vth=%f", vx, vth);
  }

  void publishOdometry() {
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.orientation.z = sin(theta_ / 2.0);
    msg.pose.pose.orientation.w = cos(theta_ / 2.0);

    msg.twist.twist.linear.x =
        (wheel_radius_ / 2.0) * (wheel_velocity_left_ + wheel_velocity_right_);
    msg.twist.twist.angular.z = (wheel_radius_ / wheelbase_) *
                                (wheel_velocity_right_ - wheel_velocity_left_);
    odom_publisher_->publish(msg);
  }

  void update() {
    robot_->step(time_step_);

    if (sensor_) {
      double distance = sensor_->getValue();
      RCLCPP_INFO(this->get_logger(), "Distance: %.3f", distance);
    }

    rclcpp::Time now = this->now();
    if (now - latest_rps_updated_ >
        rclcpp::Duration(cmd_vel_duration_sec_, cmd_vel_duration_usec_)) {
      RCLCPP_INFO(this->get_logger(), "Command expired. Stop.");
      target_rps_left_ = 0.0;
      target_rps_right_ = 0.0;
    }

    if (motor_left_) {
      motor_left_->setVelocity(target_rps_left_);
    }
    if (motor_right_) {
      motor_right_->setVelocity(target_rps_right_);
    }

    double delta_left = 0.0;
    double delta_right = 0.0;
    if (position_sensor_left_) {
      wheel_position_left_ = position_sensor_left_->getValue();
      if (std::isnan(wheel_position_left_prev_)) {
        wheel_position_left_prev_ = wheel_position_left_;
      }
      delta_left = wheel_position_left_ - wheel_position_left_prev_;

      wheel_position_left_prev_ = wheel_position_left_;
      RCLCPP_INFO(this->get_logger(), "Left Wheel Position: %.3f",
                  wheel_position_left_);
    }
    if (position_sensor_right_) {
      wheel_position_right_ = position_sensor_right_->getValue();
      if (std::isnan(wheel_position_right_prev_)) {
        wheel_position_right_prev_ = wheel_position_right_;
      }
      delta_right = wheel_position_right_ - wheel_position_right_prev_;
      wheel_position_right_prev_ = wheel_position_right_;
      RCLCPP_INFO(this->get_logger(), "Right Wheel Position: %.3f",
                  wheel_position_right_);
    }

    double odom_translation = (delta_left + delta_right) * wheel_radius_ / 2.0;
    double odom_rotation =
        (delta_right - delta_left) * wheel_radius_ / wheelbase_;
    x_ += odom_translation * cos(theta_ + odom_rotation / 2);
    y_ += odom_translation * sin(theta_ + odom_rotation / 2);
    theta_ += odom_rotation;
    wheel_velocity_left_ = delta_left * time_step_;
    wheel_velocity_right_ = delta_right * time_step_;
    publishOdometry();
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebotsRos2Node>());
  rclcpp::shutdown();
  return 0;
}
