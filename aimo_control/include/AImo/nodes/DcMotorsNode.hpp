#pragma once

#include <memory>
#include <string>
#include <gpiod.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "aimo_msgs/msg/motor_speed_command.hpp"
#include "aimo_control/helpers/Pid.hpp"

struct ChipCloser {
    void operator()(gpiod_chip* c) const {
        if (c) gpiod_chip_close(c);
    }
};

struct LineReleaser {
    void operator()(gpiod_line* l) const {
        if (l) 
        {   
            gpiod_line_set_value(l, 0);
            gpiod_line_release(l);
        }
    }
};

using ChipPtr = std::unique_ptr<gpiod_chip, ChipCloser>;
using LinePtr = std::unique_ptr<gpiod_line, LineReleaser>;

class DcMotor: public rclcpp::Node
{
    public:
        DcMotor(rclcpp::NodeOptions options);
        ~DcMotor();
    private:
        void setupPinOut(LinePtr& line, const int pin, const std::string& name);
        void setupPinIn(LinePtr& line, const int pin, const std::string& name);
        void timerCallback();
        void speedCallback(const aimo_msgs::msg::MotorSpeedCommand::SharedPtr msg);
        void motorThread(LinePtr &pwm, LinePtr &pin1, LinePtr &pin2, cost float pwmValue);
        ChipPtr chip_;
        LinePtr mA1_, mB1_, mA2_, mB2_, pwmA_, pwmB_, standby_;
        LinePtr eA1_, eA2_, eB1_, eB2_;
        int encoderAllTicks_;
        float wantedSpeedMotor1_;
        float wantedSpeedMotor2_;
        float currentSpeedMotor1_;
        float currentSpeedMotor2_;
        aimo_msgs::msg::MotorSpeedCommand motorsWantedSpeed_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<aimo_msgs::msg::MotorSpeedCommand>::SharedPtr publisherCurrentSpeed_;
        rclcpp::Subscription<aimo_msgs::msg::MotorSpeedCommand>::SharedPtr subscriptionMotorSpeed_;
        std::unique_ptr<PIDRegulator> pidA_, pidB_;
        int lastTicksA_, lastTicksA_;
        std::std::unique_ptr<std::thread> threadMotorA_, threadMotorB_;
        std::atomic<float> pwmFillA_{0.0};
        std::atomic<float> pwmFillB_{0.0};
        std::atomic<bool> running_{true};
        std::std::unique_ptr<std::thread> threadEncoderA_, threadEncoderB_;
        std::atomic<int> tickEncoderA_{0}, tickEncoderB_{0};
}
