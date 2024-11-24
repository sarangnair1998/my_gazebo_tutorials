/**
 * @file walker.cpp
 *
 * Copyright (c) 2024 Sarang Shibu
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "walker.hpp"

/**
 * @brief Executes the behavior for the ForwardState.
 * Moves the robot forward until an obstacle is detected.
 */
void ForwardState::execute(Walker* context) {
  auto cmd = geometry_msgs::msg::Twist();

  if (context->is_obstacle_detected()) {
    // Stop the robot and transition to turning state
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    if (context->last_turn_clockwise) {
      RCLCPP_INFO(context->get_logger(),
                  "Obstacle detected. Transitioning to AnticlockwiseState.");
      context->set_state(std::make_shared<AnticlockwiseState>());
      context->last_turn_clockwise = false;  // Toggle direction
    } else {
      RCLCPP_INFO(context->get_logger(),
                  "Obstacle detected. Transitioning to ClockwiseState.");
      context->set_state(std::make_shared<ClockwiseState>());
      context->last_turn_clockwise = true;  // Toggle direction
    }
  } else {
    // Move forward
    cmd.linear.x = 0.2;  // Positive linear velocity
    cmd.angular.z = 0.0;
    RCLCPP_INFO(context->get_logger(), "Moving forward.");
  }

  context->publish_velocity(cmd);
}

/**
 * @brief Executes the behavior for the ClockwiseState.
 * Turns the robot clockwise until the path ahead is clear.
 */
void ClockwiseState::execute(Walker* context) {
  auto cmd = geometry_msgs::msg::Twist();

  if (!context->is_obstacle_detected()) {
    // Path is clear, transition back to ForwardState
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    RCLCPP_INFO(context->get_logger(),
                "Path clear. Transitioning to ForwardState.");
    context->set_state(std::make_shared<ForwardState>());
  } else {
    // Continue turning clockwise
    cmd.linear.x = 0.0;
    cmd.angular.z = -0.5;  // Negative angular velocity
    RCLCPP_INFO(context->get_logger(),
                "Turning clockwise. Obstacle still detected.");
  }

  context->publish_velocity(cmd);
}

/**
 * @brief Executes the behavior for the AnticlockwiseState.
 * Turns the robot counterclockwise until the path ahead is clear.
 */
void AnticlockwiseState::execute(Walker* context) {
  auto cmd = geometry_msgs::msg::Twist();

  if (!context->is_obstacle_detected()) {
    // Path is clear, transition back to ForwardState
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    RCLCPP_INFO(context->get_logger(),
                "Path clear. Transitioning to ForwardState.");
    context->set_state(std::make_shared<ForwardState>());
  } else {
    // Continue turning counterclockwise
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.5;  // Positive angular velocity
    RCLCPP_INFO(context->get_logger(),
                "Turning anticlockwise. Obstacle still detected.");
  }

  context->publish_velocity(cmd);
}

/**
 * @brief Walker Constructor: Initializes ROS 2 Node and sets default state.
 */
Walker::Walker()
    : Node("walker_node"),
    last_turn_clockwise(true),  // Initialize in the same order as declared
      current_state_(std::make_shared<ForwardState>()),
      obstacle_detected_(false) {
  RCLCPP_INFO(this->get_logger(), "Initializing Walker Node");

  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  RCLCPP_INFO(this->get_logger(), "Velocity publisher created");

  laser_sub_ = this->create_subscription
  <sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&Walker::laser_callback,
      this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "LaserScan subscriber created");

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&Walker::execute, this));
  RCLCPP_INFO(this->get_logger(), "Timer created");
}


/**
 * @brief Publish velocity commands to the robot.
 */
void Walker::publish_velocity(const geometry_msgs::msg::Twist& cmd) {
  vel_pub_->publish(cmd);
}

/**
 * @brief Checks if an obstacle is detected.
 */
bool Walker::is_obstacle_detected() const {
  return obstacle_detected_;
}

/**
 * @brief Sets the current state of the Walker.
 */
void Walker::set_state(std::shared_ptr<State> state) {
  current_state_ = state;
}

/**
 * @brief Gets the logger instance of this node.
 */
rclcpp::Logger Walker::get_logger() const {
  return Node::get_logger();
}

/**
 * @brief Callback function for LaserScan messages.
 * Updates the obstacle detection flag.
 */
void Walker::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Reset obstacle detection
  obstacle_detected_ = false;

  // Check first and last 60 points for obstacles
  const int range_to_check = 60;

  for (size_t i = 0; i < range_to_check; ++i) {
    if (msg->ranges[i] < 0.5 ||
        msg->ranges[msg->ranges.size() - 1 - i] < 0.5) {  // Threshold: 0.5m
      obstacle_detected_ = true;
      break;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Obstacle detected: %s",
              obstacle_detected_ ? "Yes" : "No");
}

/**
 * @brief Executes the behavior of the current state.
 */
void Walker::execute() {
  if (current_state_) {
    current_state_->execute(this);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Current state is null!");
  }
}

/**
 * @brief Main function to run the Walker node.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("walker_main"), "Starting Walker Node...");
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}
