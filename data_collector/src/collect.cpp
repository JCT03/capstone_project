#include <stdio.h>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "irobot_create_msgs/msg/ir_intensity_vector.hpp"
#include "irobot_create_msgs/msg/mouse.hpp"
#include "irobot_create_msgs/msg/wheel_vels.hpp"

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
    }
  private:
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr _hazardSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imuSubscriber;
    rclcpp::Subscription<irobot_create_msgs::msg::IrIntensityVector>::SharedPtr _irSubscriber;
    rclcpp::Subscription<irobot_create_msgs::msg::Mouse>::SharedPtr _mouseSubscriber;
    rclcpp::Subscription<irobot_create_msgs::msg::WheelVels>::SharedPtr _wheelSubscriber;
    bool collision_occured = false;
    std::array<double, 21> sensor_data{};
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
          for (double val : sensor_data) {
            std::cout << val << std::endl;
          }
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
