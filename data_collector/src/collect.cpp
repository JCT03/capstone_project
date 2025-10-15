#include <stdio.h>
#include <array>
#include <fstream>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "irobot_create_msgs/msg/ir_intensity_vector.hpp"
#include "irobot_create_msgs/msg/mouse.hpp"
#include "irobot_create_msgs/msg/wheel_vels.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "irobot_create_msgs/srv/reset_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"

class Collect : public rclcpp::Node {
  public:
    Collect() : Node("collect") {
      _hazardSubscriber = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
        "hazard_detection", rclcpp::QoS(10).best_effort(), std::bind(&Collect::hazard_callback, this,
        std::placeholders::_1));
      _imuSubscriber = this->create_subscription<sensor_msgs::msg::Imu>("imu",
         rclcpp::QoS(10).best_effort(), std::bind(&Collect::imu_callback, this, std::placeholders::_1));
      _irSubscriber = this->create_subscription<irobot_create_msgs::msg::IrIntensityVector>(
        "ir_intensity", rclcpp::QoS(10).best_effort(), std::bind(&Collect::ir_callback, this,
        std::placeholders::_1));
      _mouseSubscriber = this->create_subscription<irobot_create_msgs::msg::Mouse>("mouse",
        rclcpp::QoS(10).best_effort(), std::bind(&Collect::mouse_callback, this, std::placeholders::_1));
      _wheelSubscriber = this->create_subscription<irobot_create_msgs::msg::WheelVels>("wheel_vels",
        rclcpp::QoS(10).best_effort(), std::bind(&Collect::wheel_callback, this, std::placeholders::_1));
      _inputSubscriber = this->create_subscription<std_msgs::msg::String>("input", 10,
        std::bind(&Collect::input_callback, this, std::placeholders::_1));
      _odomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(10).best_effort(), std::bind(&Collect::odom_callback, this, std::placeholders::_1));
      _outputPublisher = this->create_publisher<std_msgs::msg::String>("output", 10);
      _poseClient = this->create_client<irobot_create_msgs::srv::ResetPose>("reset_pose");
      std_msgs::msg::String s = std_msgs::msg::String();
      s.data = "Waiting for collision, press 'r' to reset odometry";
      _outputPublisher->publish(s);
    }
  private:
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr _hazardSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imuSubscriber;
    rclcpp::Subscription<irobot_create_msgs::msg::IrIntensityVector>::SharedPtr _irSubscriber;
    rclcpp::Subscription<irobot_create_msgs::msg::Mouse>::SharedPtr _mouseSubscriber;
    rclcpp::Subscription<irobot_create_msgs::msg::WheelVels>::SharedPtr _wheelSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _inputSubscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _outputPublisher;
    rclcpp::Client<irobot_create_msgs::srv::ResetPose>::SharedPtr _poseClient;
    bool collision_occured = false;
    std::array<double, 22> sensor_data{};
    std::array<double, 3> odom_offset{};
    double odomX_ = 0.0;
    double odomY_ = 0.0;
    double odomTheta_ = 0.0;
    double originX_ = 0.0;
    double originY_ = 0.0;
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
      odomX_ = msg->pose.pose.position.x;
      odomY_ = msg->pose.pose.position.y;
      tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
      );
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      odomTheta_ = yaw * 180.0 / M_PI;
      if (!collision_occured) {
        sensor_data[21] = odomTheta_;
      }
  }
    void input_callback(const std_msgs::msg::String::SharedPtr msg) {
      if (msg->data == "1") {
        odom_offset[0] = odomTheta_;
        originX_ = odomX_ * -1;
        originY_ = odomY_ * -1;
        _poseClient->async_send_request(std::make_shared<irobot_create_msgs::srv::ResetPose::Request>());
        std_msgs::msg::String s = std_msgs::msg::String();
        s.data = "Once odometry has reset, navigate to origin and press 2      ";
        _outputPublisher->publish(s);
      } else if (msg->data == "2") {
        odom_offset[1] = odomX_ - originX_;
        odom_offset[2] = odomY_ - originY_;
        _poseClient->async_send_request(std::make_shared<irobot_create_msgs::srv::ResetPose::Request>());
        std::ofstream file("collision_data.csv", std::ios::app);
        for (double d : sensor_data) {
          file << d << ",";
        }
        for (double o : odom_offset) {
          file << o << ",";
        }
        file << std::endl;
        sensor_data[0] = 0.0;
        sensor_data[1] = 0.0;
        sensor_data[2] = 0.0;
        sensor_data[3] = 0.0;
        sensor_data[4] = 0.0;
        collision_occured = false;
        std_msgs::msg::String s = std_msgs::msg::String();
        s.data = "Data recorded, waiting for next collision                 ";
        _outputPublisher->publish(s);
      } else if (msg->data == "r") {
        _poseClient->async_send_request(std::make_shared<irobot_create_msgs::srv::ResetPose::Request>());
      }
    }
    void wheel_callback(const irobot_create_msgs::msg::WheelVels::SharedPtr msg) {
      if (!collision_occured) {
        sensor_data[19] = msg->velocity_left;
        sensor_data[20] = msg->velocity_right;
      }
    }
    void mouse_callback(const irobot_create_msgs::msg::Mouse::SharedPtr msg) {
      if (!collision_occured) {
        sensor_data[18] = msg->last_squal;
      }
    }
    void ir_callback(const irobot_create_msgs::msg::IrIntensityVector::SharedPtr msg) {
      if (!collision_occured) {
        sensor_data[11] = msg->readings[0].value;
        sensor_data[12] = msg->readings[1].value;
        sensor_data[13] = msg->readings[2].value;
        sensor_data[14] = msg->readings[3].value;
        sensor_data[15] = msg->readings[4].value;
        sensor_data[16] = msg->readings[5].value;
        sensor_data[17] = msg->readings[6].value;
      }
    }
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
      if (!collision_occured) {
        sensor_data[5] = msg->linear_acceleration.x;
        sensor_data[6] = msg->linear_acceleration.y;
        sensor_data[7] = msg->linear_acceleration.z;
        sensor_data[8] = msg->angular_velocity.x;
        sensor_data[9] = msg->angular_velocity.y;
        sensor_data[10] = msg->angular_velocity.z;
      }
    }
    void hazard_callback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg) {
      if (!collision_occured) {
        for (const auto& hazard : msg->detections) {
          if (hazard.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
            if (hazard.header.frame_id == "bump_left") {
              sensor_data[0] = 1.0;
            } else if (hazard.header.frame_id == "bump_front_left") {
              sensor_data[1] = 1.0;
            } else if (hazard.header.frame_id == "bump_front_center") {
              sensor_data[2] = 1.0;
            } else if (hazard.header.frame_id == "bump_front_right") {
              sensor_data[3] = 1.0;
            } else {
              sensor_data[4] = 1.0;
            }
            collision_occured=true;
          }
        }
        if (collision_occured) {
          std_msgs::msg::String s = std_msgs::msg::String();
          s.data = "Collision detected! Turn robot to true angle 0 and press 1";
          _outputPublisher->publish(s);
        }
      }
    }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Collect>());
  rclcpp::shutdown();
  return 0;
}
