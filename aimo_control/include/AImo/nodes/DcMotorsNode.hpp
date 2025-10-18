#pragma once

#include <memory>
#include <string>
#include <gpiod.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "aimo_msgs/msg/motor_speed_command.hpp"

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
    private:
        void setupPin(LinePtr& line, const int pin, const std::string& name);
        void speedCallback(const aimo_msgs::msg::MotorSpeedCommand::SharedPtr msg)
        ChipPtr chip_;
        LinePtr mA1_;
        LinePtr mB1_;
        LinePtr mA2_;
        LinePtr mB2_;
        LinePtr pwmA_;
        LinePtr pwmB_;
        LinePtr standby_;
        int encoderAllTicks_;
        float wantedSpeedMotor1_;
        float wantedSpeedMotor2_;
        float currentSpeedMotor1_;
        float currentSpeedMotor2_;
        aimo_msgs::msg::MotorSpeedCommand motorsWantedSpeed_;
        rclcpp::Publisher<aimo_msgs::msg::MotorSpeedCommand>::SharedPtr publisherCurrentSpeed_;
        rclcpp::Subscription<aimo_msgs::msg::MotorSpeedCommand>::SharedPtr subscriptionMotorSpeed_;
}