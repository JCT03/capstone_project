#include <stdio.h>
#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <sstream>
#include <iomanip>

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
#include <torch/torch.h>

struct Net : torch::nn::Module {
  Net() {
    fc1 = register_module("fc1", torch::nn::Linear(22, 32));
    dropout1 = register_module("dropout1", torch::nn::Dropout(0.3));
    fc2 = register_module("fc2", torch::nn::Linear(32, 64));
    dropout2 = register_module("dropout2", torch::nn::Dropout(0.3));
    fc3 = register_module("fc3", torch::nn::Linear(64, 3));
  }
  torch::Tensor forward(torch::Tensor x) {
    x = torch::relu(fc1->forward(x));
    x = dropout1->forward(x);
    x = torch::relu(fc2->forward(x));
    x = dropout2->forward(x);
    x = fc3->forward(x);
    return x;
  }
  torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};
  torch::nn::Dropout dropout1{nullptr}, dropout2{nullptr};
};

class Offset : public rclcpp::Node {
  public:
    Offset() : Node("offset") {
      _hazardSubscriber = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
        "hazard_detection", rclcpp::QoS(10).best_effort(), std::bind(&Offset::hazard_callback, this,
        std::placeholders::_1));
      _imuSubscriber = this->create_subscription<sensor_msgs::msg::Imu>("imu",
         rclcpp::QoS(10).best_effort(), std::bind(&Offset::imu_callback, this, std::placeholders::_1));
      _irSubscriber = this->create_subscription<irobot_create_msgs::msg::IrIntensityVector>(
        "ir_intensity", rclcpp::QoS(10).best_effort(), std::bind(&Offset::ir_callback, this,
        std::placeholders::_1));
      _mouseSubscriber = this->create_subscription<irobot_create_msgs::msg::Mouse>("mouse",
        rclcpp::QoS(10).best_effort(), std::bind(&Offset::mouse_callback, this, std::placeholders::_1));
      _wheelSubscriber = this->create_subscription<irobot_create_msgs::msg::WheelVels>("wheel_vels",
        rclcpp::QoS(10).best_effort(), std::bind(&Offset::wheel_callback, this, std::placeholders::_1));
      _odomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(10).best_effort(), std::bind(&Offset::odom_callback, this, std::placeholders::_1));
      _outputPublisher = this->create_publisher<std_msgs::msg::String>("output", 10);
      _net = std::make_shared<Net>();
      torch::load(_net, "/home/robotics/capstone_project/networks/optimized/A.pt");
      odom_offset = {0, 0, 0};
    }
  private:
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr _hazardSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imuSubscriber;
    rclcpp::Subscription<irobot_create_msgs::msg::IrIntensityVector>::SharedPtr _irSubscriber;
    rclcpp::Subscription<irobot_create_msgs::msg::Mouse>::SharedPtr _mouseSubscriber;
    rclcpp::Subscription<irobot_create_msgs::msg::WheelVels>::SharedPtr _wheelSubscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _outputPublisher;
    bool collision_occured = false;
    std::array<double, 22> sensor_data{};
    std::array<double, 3> odom_offset{};
    std::shared_ptr<Net> _net;
    rclcpp::TimerBase::SharedPtr _timer;
    void reset_collision_state() {
	  sensor_data[0] = -0.683309421;
	  sensor_data[1] = -0.683309421;
	  sensor_data[2] = -0.683309421;
	  sensor_data[3] = -0.683309421;
  	sensor_data[4] = -0.683309421;
        collision_occured = false;
        _timer->cancel();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
      double odomX_ = msg->pose.pose.position.x;
      double odomY_ = msg->pose.pose.position.y;
      tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
      );
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      double odomTheta_ = yaw * 180.0 / M_PI;
      if (!collision_occured) {
        sensor_data[21] = (odomTheta_+5.175686471)/100.3310042;
      }
      double corrected_theta = odomTheta_ + odom_offset[0];
      corrected_theta = std::fmod(corrected_theta, 360.0);
      if (corrected_theta > 180) {
        corrected_theta -= 360;
      } else if (corrected_theta < -180) {
        corrected_theta += 360;
      }
      double corrected_x = odomX_ + odom_offset[1];
      double corrected_y = odomY_ + odom_offset[2];
      std::stringstream ss;
      ss << "Corrected Position: X: " << std::fixed << std::setprecision(2) 
        << corrected_x << ", Y: " << corrected_y << ", Yaw: " << corrected_theta << " deg     ";
      std_msgs::msg::String s = std_msgs::msg::String();
      s.data = ss.str();
      _outputPublisher->publish(s);
    }

    void wheel_callback(const irobot_create_msgs::msg::WheelVels::SharedPtr msg) {
      if (!collision_occured) {
        sensor_data[19] = (msg->velocity_left-7.569417752)/3.62661051;
        sensor_data[20] = (msg->velocity_right-7.638250103)/3.473969527;
      }
    }

    void mouse_callback(const irobot_create_msgs::msg::Mouse::SharedPtr msg) {
      if (!collision_occured) {
        sensor_data[18] = (msg->last_squal-107.0426065)/28.50345253;
      }
    }

    void ir_callback(const irobot_create_msgs::msg::IrIntensityVector::SharedPtr msg) {
      if (!collision_occured) {
        sensor_data[11] = (msg->readings[0].value-238.3784461)/515.4597672;
        sensor_data[12] = (msg->readings[1].value-665.1929825)/1094.418908;
        sensor_data[13] = (msg->readings[2].value-617.4185464)/961.5451708;
        sensor_data[14] = (msg->readings[3].value-839.6616541)/1114.868215;
        sensor_data[15] = (msg->readings[4].value-860.8295739)/1160.702961;
        sensor_data[16] = (msg->readings[5].value-657.481203)/1105.995776;
        sensor_data[17] = (msg->readings[6].value-339.0050125)/716.1085033;
      }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
      if (!collision_occured) {
        sensor_data[5] = (msg->linear_acceleration.x+0.884702229)/2.989697113;
        sensor_data[6] = (msg->linear_acceleration.y-0.0269848)/1.011681074;
        sensor_data[7] = (msg->linear_acceleration.z-0.095883377)/0.76527972;
        sensor_data[8] = (msg->angular_velocity.x+0.000608536)/0.034217143;
        sensor_data[9] = (msg->angular_velocity.y+0.000023298)/0.078649704;
        sensor_data[10] = (msg->angular_velocity.z-0.0297538)/0.479763795;
      }
    }

    void hazard_callback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg) {
      if (!collision_occured) {
        for (const auto& hazard : msg->detections) {
          if (hazard.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
            if (hazard.header.frame_id == "bump_left") {
              sensor_data[0] = 1.463465846;
            } else if (hazard.header.frame_id == "bump_front_left") {
              sensor_data[1] = 1.463465846;
            } else if (hazard.header.frame_id == "bump_front_center") {
              sensor_data[2] = 1.463465846;
            } else if (hazard.header.frame_id == "bump_front_right") {
              sensor_data[3] = 1.463465846;
            } else {
              sensor_data[4] = 1.463465846;
            }
            collision_occured=true;
            _net->eval();
            torch::NoGradGuard no_grad;
            torch::Tensor input_tensor = torch::from_blob(sensor_data.data(),{1,22},torch::kDouble).to(torch::kFloat);;
            torch::Tensor output_tensor = _net->forward(input_tensor);
            float* output_data_ptr = output_tensor.data_ptr<float>();
            odom_offset[0] += static_cast<double>(output_data_ptr[0])*2.725697775+0.054030836;
            odom_offset[1] += static_cast<double>(output_data_ptr[1])*0.117770977-0.01201306;
            odom_offset[2] += static_cast<double>(output_data_ptr[2])*0.172681628-0.010258537;
            _timer = this->create_wall_timer(
                std::chrono::seconds(3),
                std::bind(&Offset::reset_collision_state, this)
            );
          }
        }
      }
    }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Offset>());
  rclcpp::shutdown();
  return 0;
}
