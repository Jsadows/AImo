#include "aimo_control/nodes/DcMotorsNode.hpp"

DcMotor::DcMotor(rclcpp::NodeOptions options) : Node("motors", options)
{
    this->declare_parameter("motorAOutPin1", rclcpp::ParameterValue(26));
    this->declare_parameter("motorAOutPin2", rclcpp::ParameterValue(19));
    this->declare_parameter("motorBOutPin1", rclcpp::ParameterValue(20));
    this->declare_parameter("motorBOutPin2", rclcpp::ParameterValue(21));
    this->declare_parameter("motorAPWMPin", rclcpp::ParameterValue(13));
    this->declare_parameter("motorBPWMPin", rclcpp::ParameterValue(12));
    this->declare_parameter("standbyPin", rclcpp::ParameterValue(16));
    this->declare_parameter("encoderAllTicks", rclcpp::ParameterValue(960));
    this->declare_parameter("chipname", rclcpp::ParameterValue("gpiochip0"));

    int motorAOutPin1 = this->get_parameter("motorAOutPin1").as_int();
    int motorAOutPin2 = this->get_parameter("motorAOutPin2").as_int();
    int motorBOutPin1 = this->get_parameter("motorBOutPin1").as_int();
    int motorBOutPin2 = this->get_parameter("motorBOutPin2").as_int();
    int motorAPWMPin = this->get_parameter("motorAPWMPin").as_int();
    int motorBPWMPin = this->get_parameter("motorBPWMPin").as_int();
    int standbyPin = this->get_parameter("standbyPin").as_int();
    int encoderAllTicks = this->get_parameter("encoderAllTicks").as_int();
    std::string chipname = this->get_parameter("chipname").as_string();

    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorAOutPin1 " << motorAOutPin1);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorAOutPin2 " << motorAOutPin2);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorBOutPin1 " << motorBOutPin1);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorBOutPin2 " << motorBOutPin2);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorAPWMPin " << motorAPWMPin);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorBPWMPin " << motorBPWMPin);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: standbyPin " << standbyPin);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: encoderAllTicks " << encoderAllTicks);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: chipname " << chipname);

    chip_ =  = gpiod_chip_open(chipname.c_str());
    etupPin(mA1_, motorAOutPin1, "m1A");
    setupPin(mB1_, motorBOutPin1, "m1B");
    setupPin(mA2_, motorAOutPin2, "m2A");
    setupPin(mB2_, motorBOutPin2, "m2B");
    setupPin(pwm1_, motorAPWMPin, "pwm1");
    setupPin(pwm2_, motorBPWMPin, "pwm2");
    setupPin(standby_, standbyPin, "standby");

    publisherCurrentSpeed_ =
      this->create_publisher<aimo_msgs::msg::MotorSpeedCommand>("/current_speed", 10);

    subscriptionMotorSpeed_ = this->create_subscription<aimo_msgs::msg::MotorSpeedCommand>(
      "/motors_speed", 10,
      std::bind(&DcMotor::speedCallback, this, std::placeholders::_1));

}

void DcMotor::setupPin(LinePtr line, int pin, std::string& name)
{
    line.reset(gpiod_chip_get_line(chip_.get(), pin));
    if (line == nullptr) throw std::runtime_error("Nie mogę pobrać linii GPIO");
    if (gpiod_line_request_output(line, name.c_str(), 0) < 0) {
        throw std::runtime_error("Nie mogę ustawić linii jako wyjście");
    }
}

void DcMotor::speedCallback(const aimo_msgs::msg::MotorSpeedCommand::SharedPtr msg)
{
  motorsWantedSpeed_ = *msg;
}