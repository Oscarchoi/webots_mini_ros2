#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

using namespace webots;

class WebotsRos2Node : public rclcpp::Node {
 public:
  WebotsRos2Node() : Node("webots_ros2_node") {
    RCLCPP_INFO(this->get_logger(), "Starting Webots ROS2 Node...");

    // Initialize Webots Object
    robot_ = new Robot();

    // Initialize Webots Device
    sensor_ = robot_->getDistanceSensor("ds0");
    if (sensor_) {
      sensor_->enable(TIME_STEP);
      RCLCPP_INFO(this->get_logger(), "Found distance sensor: %s", "ds0");
    }

    this->declare_parameter<std::string>("left_wheel_motor",
                                         "left wheel motor");
    std::string left_wheel_motor =
        this->get_parameter("left_wheel_motor").as_string();

    motor_left_ = robot_->getMotor(left_wheel_motor);
    if (motor_left_) {
      motor_left_->setPosition(INFINITY);
      motor_left_->setVelocity(1.0);
      RCLCPP_INFO(this->get_logger(), "Found motor: '%s'",
                  left_wheel_motor.c_str());
    }
    position_sensor_left_ = motor_left_->getPositionSensor();
    if (position_sensor_left_) {
      position_sensor_left_->enable(TIME_STEP);
      RCLCPP_INFO(this->get_logger(), "Found position sensor of motor: '%s'",
                  left_wheel_motor.c_str());
    }

    this->declare_parameter<std::string>("right_wheel_motor",
                                         "right wheel motor");
    std::string right_wheel_motor =
        this->get_parameter("right_wheel_motor").as_string();

    motor_right_ = robot_->getMotor(right_wheel_motor);
    if (motor_right_) {
      motor_right_->setPosition(INFINITY);
      motor_right_->setVelocity(1.0);
      RCLCPP_INFO(this->get_logger(), "Found motor: '%s'",
                  right_wheel_motor.c_str());
    }
    position_sensor_right_ = motor_right_->getPositionSensor();
    if (position_sensor_right_) {
      position_sensor_right_->enable(TIME_STEP);
      RCLCPP_INFO(this->get_logger(), "Found position sensor of motor: '%s'",
                  right_wheel_motor.c_str());
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(TIME_STEP),
                                     std::bind(&WebotsRos2Node::update, this));
  }

  ~WebotsRos2Node() { delete robot_; }

 private:
  static constexpr int TIME_STEP = 1000;

  Robot* robot_;

  Motor* motor_left_;
  PositionSensor* position_sensor_left_;

  Motor* motor_right_;
  PositionSensor* position_sensor_right_;

  DistanceSensor* sensor_;
  rclcpp::TimerBase::SharedPtr timer_;

  void update() {
    robot_->step(TIME_STEP);

    if (sensor_) {
      double distance = sensor_->getValue();
      RCLCPP_INFO(this->get_logger(), "Distance: %.3f", distance);
    }

    if (position_sensor_left_) {
      double position = position_sensor_left_->getValue();
      RCLCPP_INFO(this->get_logger(), "Left Wheel Position: %.3f", position);
    }
    if (position_sensor_right_) {
      double position = position_sensor_right_->getValue();
      RCLCPP_INFO(this->get_logger(), "Right Wheel Position: %.3f", position);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebotsRos2Node>());
  rclcpp::shutdown();
  return 0;
}
