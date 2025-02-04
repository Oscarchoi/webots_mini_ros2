#include <memory>

#include <geometry_msgs/msg/twist.hpp>
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

    auto command_topic =
        get_or_default_parameter(this, "command_topic", std::string("cmd_vel"));
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

  double target_rps_left_;
  double target_rps_right_;
  double wheel_position_left_;
  double wheel_position_right_;
  rclcpp::Time latest_rps_updated_;

  void onCommand(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double vx = msg->linear.x;
    double vth = msg->angular.z;
    target_rps_left_ = (vx - (vth * wheelbase_ / 2.0)) / wheel_radius_;
    target_rps_right_ = (vx + (vth * wheelbase_ / 2.0)) / wheel_radius_;
    latest_rps_updated_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Received command: vx=%f, vth=%f", vx, vth);
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

    if (position_sensor_left_) {
      wheel_position_left_ = position_sensor_left_->getValue();
      RCLCPP_INFO(this->get_logger(), "Left Wheel Position: %.3f",
                  wheel_position_left_);
    }
    if (position_sensor_right_) {
      wheel_position_right_ = position_sensor_right_->getValue();
      RCLCPP_INFO(this->get_logger(), "Right Wheel Position: %.3f",
                  wheel_position_right_);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebotsRos2Node>());
  rclcpp::shutdown();
  return 0;
}
