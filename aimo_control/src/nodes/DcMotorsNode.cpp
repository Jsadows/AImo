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
  this->declare_parameter("encoderA1Pin", rclcpp::ParameterValue(16));
  this->declare_parameter("encoderA2Pin", rclcpp::ParameterValue(16));
  this->declare_parameter("encoderB1Pin", rclcpp::ParameterValue(16));
  this->declare_parameter("encoderB2Pin", rclcpp::ParameterValue(16));
  this->declare_parameter("encoderAllTicks", rclcpp::ParameterValue(960));
  this->declare_parameter("chipname", rclcpp::ParameterValue("gpiochip0"));
  this->declare_parameter("k", rclcpp::ParameterValue(1.0));
  this->declare_parameter("ti", rclcpp::ParameterValue(100000000.0));
  this->declare_parameter("td", rclcpp::ParameterValue(0.0));
  this->declare_parameter("timerPeriod", rclcpp::ParameterValue(100));
  this->declare_parameter("motorPeriodUs", rclcpp::ParameterValue(10000));

  const int motorAOutPin1 = this->get_parameter("motorAOutPin1").as_int();
  const int motorAOutPin2 = this->get_parameter("motorAOutPin2").as_int();
  const int motorBOutPin1 = this->get_parameter("motorBOutPin1").as_int();
  const int motorBOutPin2 = this->get_parameter("motorBOutPin2").as_int();
  const int motorAPWMPin = this->get_parameter("motorAPWMPin").as_int();
  const int motorBPWMPin = this->get_parameter("motorBPWMPin").as_int();
  const int standbyPin = this->get_parameter("standbyPin").as_int();
  const int encoderA1Pin = this->get_parameter("encoderA1Pin").as_int();
  const int encoderA2Pin = this->get_parameter("encoderA2Pin").as_int();
  const int encoderB1Pin = this->get_parameter("encoderB1Pin").as_int();
  const int encoderB2Pin = this->get_parameter("encoderB2Pin").as_int();
  const int encoderAllTicks = this->get_parameter("encoderAllTicks").as_int();
  const std::string chipname = this->get_parameter("chipname").as_string();
  const float k  = static_cast<float>(this->get_parameter("k").as_double());
  const float ti = static_cast<float>(this->get_parameter("ti").as_double());
  const float td = static_cast<float>(this->get_parameter("td").as_double());
  timerPeriod_ = this->get_parameter("timerPeriod").as_int();
  const int motorPeriodUs = this->get_parameter("motorPeriodUs").as_int();

  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorAOutPin1 " << motorAOutPin1);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorAOutPin2 " << motorAOutPin2);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorBOutPin1 " << motorBOutPin1);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorBOutPin2 " << motorBOutPin2);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorAPWMPin " << motorAPWMPin);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorBPWMPin " << motorBPWMPin);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: standbyPin " << standbyPin);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: encoderA1Pin " << encoderA1Pin);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: encoderA2Pin " << encoderA2Pin);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: encoderB1Pin " << encoderB1Pin);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: encoderB2Pin " << encoderB2Pin);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: encoderAllTicks " << encoderAllTicks);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: chipname " << chipname);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: k " << k);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: ti " << ti);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: td " << td);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timerPeriod " << timerPeriod);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: motorPeriodUs " << motorPeriodUs);

  chip_ =  = gpiod_chip_open(chipname.c_str());
  setupPinOut(mA1_, motorAOutPin1, "m1A");
  setupPinOut(mB1_, motorBOutPin1, "m1B");
  setupPinOut(mA2_, motorAOutPin2, "m2A");
  setupPinOut(mB2_, motorBOutPin2, "m2B");
  setupPinOut(pwmA_, motorAPWMPin, "pwmA");
  setupPinOut(pwmB_, motorBPWMPin, "pwmB");
  setupPinOut(standby_, standbyPin, "standby");
  setupPinIn(eA1_, encoderA1Pin, "eA1");
  setupPinIn(eA2_, encoderA2Pin, "eA2");
  setupPinIn(eB1_, encoderB1Pin, "eB1");
  setupPinIn(eB2_, encoderB2Pin, "eB2");
  gpiod_line_set_value(standby_.get(), 1);
  lastTicksA_ = 0;
  lastTicksB_ = 0;
  publisherCurrentSpeed_ =
    this->create_publisher<aimo_msgs::msg::MotorSpeedCommand>("/current_speed", 10);

  subscriptionMotorSpeed_ = this->create_subscription<aimo_msgs::msg::MotorSpeedCommand>(
    "/motors_speed", 10,
    std::bind(&DcMotor::speedCallback, this, std::placeholders::_1));

  pidA_ = make_unique<PIDRegulator>(t, k, td, ti);
  pidB_ = make_unique<PIDRegulator>(t, k, td, ti);
  threadEncoderA_ = std::make_unique<std::thread>(
    &DcMotor::encoderHandler, this, std::ref(eA1_), std::ref(eA2_), std::ref(tickEncoderA_));
  threadEncoderB_ = std::make_unique<std::thread>(
    &DcMotor::encoderHandler, this, std::ref(eB1_), std::ref(eB2_), std::ref(tickEncoderB_));
  threadMotorA_ = std::make_unique<std::thread>(&DcMotor::motorThread, this, pwm1_, mA1_, mA2_, std::ref(pwmFillA_), motorPeriodUs);
  threadMotorB_ = std::make_unique<std::thread>(&DcMotor::motorThread, this, pwm2_, mB1_, mB2_, std::ref(pwmFillB_), motorPeriodUs);
  timer_ = this->create_wall_timer(std::chrono::duration<double>(timerPeriod_),
                                  std::bind(&DcMotor::timerCallback, this));
}

DcMotor::~DcMotor()
{
    running_ = false;
    if (threadMotorA_ && threadMotorA_.joinable()) threadMotorA_.join();
    if (threadMotorB_ && threadMotorB_.joinable()) threadMotorB_.join();
    if (threadEncoderA_ && threadEncoderA_->joinable()) threadEncoderA_->join();
    if (threadEncoderB_ && threadEncoderB_->joinable()) threadEncoderB_->join();
}

void DcMotor::setupPinOut(LinePtr line, int pin, std::string& name)
{
  line.reset(gpiod_chip_get_line(chip_.get(), pin));
  if (line == nullptr) throw std::runtime_error("Nie mogę pobrać linii GPIO");
  if (gpiod_line_request_output(line, name.c_str(), 0) < 0) {
      throw std::runtime_error("Nie mogę ustawić linii jako wyjście");
  }
}

void DcMotor::setupPinIn(LinePtr line, int pin, std::string& name)
{
  line.reset(gpiod_chip_get_line(chip_.get(), pin));
  if (line == nullptr) throw std::runtime_error("Nie mogę pobrać linii GPIO");
  if (gpiod_line_request_input(line, name.c_str()) < 0) {
      throw std::runtime_error("Nie mogę ustawić linii jako wyjście");
  }
}

void DcMotor::setupPinIn(LinePtr line, int pin, std::string& name)
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

void DcMotor::timerCallback()
{
  //TODO
  int currentTicksA = tickEncoderA_.load();
  int currentTicksB = tickEncoderB_.load();

  const int deltaA = currentTicksA - lastTicksA_;
  const int deltaB = currentTicksB - lastTicksB_;
  lastTicksA_ = currentTicksA;
  lastTicksB_ = currentTicksB;
  const float omegaA = (2.0f * M_PI * deltaA / encoderAllTicks_) / timerPeriod_ * 1000.0f;
  const float omegaB = (2.0f * M_PI * deltaB / encoderAllTicks_) / timerPeriod_ * 1000.0f;
  float pwmAFill = motorsWantedSpeed_->left_motor_angular_velocity == 0.0f ? 0.0f : 
    pidA_->pid(omegaA, motorsWantedSpeed_->left_motor_angular_velocity);
  float pwmBFill = motorsWantedSpeed_->right_motor_angular_velocity == 0.0f ? 0.0f : 
    pidB_->pid(omegaB, motorsWantedSpeed_->right_motor_angular_velocity);
  pwmAFillNew = std::clamp(pwmAFill, -1.0, 1.0);
  pwmBFillNew = std::clamp(pwmBFill, -1.0, 1.0);
  pwmFillA_.store(pwmAFillNew);
  pwmFillB_.store(pwmBFillNew);
  aimo_msgs::msg::MotorSpeedCommand msg;
  msg.left_motor_angular_velocity = omegaA;
  msg.right_motor_angular_velocity = omegaB;
  publisherCurrentSpeed_->publish(msg);
}

void DcMotor::motorThread(LinePtr &pwm, LinePtr &pin1, LinePtr &pin2, std::atomic<float> pwmValue, const int period)
{
  while (running_) 
  {
    const int direction = pwmValue >= 0 ? 1 : 0;
    const float duty = std::fabs(pwmValue);
    gpiod_line_set_value(pin1.get(), direction);
    gpiod_line_set_value(pin2.get(), 1 - direction);
    const int on_time = static_cast<int>((float) period * duty);
    gpiod_line_set_value(pwm.get(), 1);
    std::this_thread::sleep_for(std::chrono::microseconds(on_time));
    gpiod_line_set_value(pwm.get(), 0);
    std::this_thread::sleep_for(std::chrono::microseconds(period - on_time));
  }
}

void DcMotor::encoderHandler(LinePtr lineA, LinePtr lineB, std::atomic<int>& ticks)
{
  int prevState = (gpiod_line_get_value(lineA.get()) << 1) |
                  gpiod_line_get_value(lineB.get());
  gpiod_line_event event;
  struct gpiod_line_bulk bulk;
  gpiod_line_bulk_init(&bulk);
  gpiod_line_bulk_add(&bulk, lineA.get());
  gpiod_line_bulk_add(&bulk, lineB.get());

  while (running_) 
  {
    if (gpiod_line_event_wait_bulk(&bulk, nullptr, &bulk) > 0) {
      for (unsigned int i = 0; i < gpiod_line_bulk_num_lines(&bulk); ++i) {
        struct gpiod_line *activeLine = gpiod_line_bulk_get_line(&bulk, i);
        if (gpiod_line_event_read(activeLine, &event) == 0) {
            int a = gpiod_line_get_value(lineA.get());
            int b = gpiod_line_get_value(lineB.get());
            int state = (a << 1) | b;

            switch ((prevState << 2) | state) {
                case 0b0001:
                case 0b0111:
                case 0b1110:
                case 0b1000:
                    ticks++;
                    break;
                case 0b0010:
                case 0b0100:
                case 0b1101:
                case 0b1011:
                    ticks--;
                    break;
                default:
                    RCLCPP_WARN(get_logger(), "Possible missed or invalid step detected!");
                    break;
            }
            prevState = state;
        }
      }
    }
  }
}
