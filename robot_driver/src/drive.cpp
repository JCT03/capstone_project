#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <chrono>
#include <thread>
#include <map>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_msgs/msg/string.hpp"

static struct termios oldt;

// Function to set terminal settings
void setTerminalMode() {
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Read input immediately and do not type to term

    // Makes input non-blocking (no min characters or time)
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

// Function to reset terminal settings
void restoreTerminalMode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

// Node to control robot movement and publish inputs to topic
class Drive : public rclcpp::Node {
public:
  Drive() : Node("drive") {
    // Initializes publishers to input and cmd_vel
    _inputPublisher = this->create_publisher<std_msgs::msg::String>("input", 10);
    _twistPublisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscribes to odom and output and binds to callbacks
    _odomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(10).best_effort(), std::bind(&Drive::odom_callback, this,
        std::placeholders::_1));
   _outputSubscriber=this->create_subscription<std_msgs::msg::String>("output", 10,
     std::bind(&Drive::output_callback, this, std::placeholders::_1));

    // Bindings to attach input keys to Twist commands
    moveBindings = {
      {'i', {1, 0, 0, 0}},
      {'o', {1, 0, 0, -1}},
      {'j', {0, 0, 0, 1}},
      {'l', {0, 0, 0, -1}},
      {'u', {1, 0, 0, 1}},
      {',', {-1, 0, 0, 0}},
      {'.', {-1, 0, 0, 1}},
      {'m', {-1, 0, 0, -1}},
      {'k', {0, 0, 0, 0}},
    };
    speedBindings = {
      {'q', {1.1, 1.1}},
      {'z', {0.9, 0.9}},
      {'w', {1.1, 1}},
      {'x', {0.9, 1}},
      {'e', {1, 1.1}},
      {'c', {1, 0.9}}
    };
  }

  void run_node() {
    // Prints instructions to terminal
    const char* msg = R"(
Robot can now be controlled via keyboard
---------------------------
Controls:
   u   i   o
   j       l
   m   ,   .
---------------------------
q/z : increase/decrease speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit\n
)";
    printf("%s", msg);
    // Calls function to set terminal mode
    setTerminalMode();

    // While there is no issue with ROS 2
    while(rclcpp::ok()) {
      // Checks if there is data to be read from standerd input
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(STDIN_FILENO, &fds);
      struct timeval tv = {0L, 0L}; // No wait time
      int result = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);

      // Variable to store input key
      int key_ = -1;

      // If there is input
      if (result > 0) {
        // Sets key to input
        key_ = getchar();
      }

      // If there is an input
      if (key_ != -1) {
        // If the input corresponds to a move binding
        if (moveBindings.count(key_) == 1) {
          // Create a twist message and set attributes
          geometry_msgs::msg::Twist twist;
          twist.linear.x = moveBindings[key_][0] * speed_;
          twist.linear.y = moveBindings[key_][1] * speed_;
          twist.linear.z = moveBindings[key_][2] * speed_;
          twist.angular.x = 0;
          twist.angular.y = 0;
          twist.angular.z = moveBindings[key_][3] * turn_;
          
          // Publish to cmd_vel topic to be read by robot
          _twistPublisher->publish(twist);
        } else {
          // If input corresponds to speed binding
          if (speedBindings.count(key_) == 1) {
            // Updates speeds appropriately
            speed_ = speed_ * speedBindings[key_][0];
            speed_ = (speed_ >= 0.47) ? 0.47 : speed_;
            speed_ = (speed_ <= 0.01) ? 0.01 : speed_;
            turn_ = turn_ * speedBindings[key_][1];
            turn_ = (turn_ >= 4.5) ? 4.5 : turn_;
            turn_ = (turn_ <= 0.1) ? 0.1 : turn_;
          // If input is control c
          } else if (key_ == '\x03') {
            // Break loop
            break;
          } else {
            // All other inputs published to input topic to be read by other nodes
            std_msgs::msg::String c = std_msgs::msg::String();
            c.data = key_;
            _inputPublisher->publish(c);
          }
        }
      }
      // Spins node to run functions
      rclcpp::spin_some(this->get_node_base_interface());

      // Prints current odometry
      printf("\rCurrent Position: X: %.2f, Y: %.2f, Yaw: %.2f deg     ",
        odomX_, odomY_, odomTheta_);
      fflush(stdout);
    }
    restoreTerminalMode();
  }

private:
  // Declares necessary variables
  std::map<char, std::vector<float>> moveBindings;
  std::map<char, std::vector<float>> speedBindings;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _twistPublisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _inputPublisher;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscriber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _outputSubscriber;
  float speed_ = 0.25;
  float turn_ = 0.75;
  double odomX_ = 0.0;
  double odomY_ = 0.0;
  double odomTheta_ = 0.0;

  // Callback to print messages read on output node to terminal
  void output_callback(const std_msgs::msg::String::SharedPtr msg) {
    printf("\n%s\033[F", (msg->data).c_str());
  }

  // Callback to update odometry values locally
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
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Drive>();
  node->run_node();
  rclcpp::shutdown();
  return 0;
}
