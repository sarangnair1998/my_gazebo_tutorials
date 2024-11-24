/**
 * @file walker.hpp
 *
 * @brief Header file for the walker application implementing the State Design Pattern.
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

#ifndef SRC_WALKER_INCLUDE_WALKER_HPP_
#define SRC_WALKER_INCLUDE_WALKER_HPP_

#include <memory>  // C++ standard library headers

#include <rclcpp/rclcpp.hpp>  // ROS headers
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

/**
 * @brief Abstract base class for the State Design Pattern.
 */
class State {
 public:
  virtual ~State() = default;

  /**
   * @brief Execute the behavior of the current state.
   * @param context Pointer to the Walker context managing the state.
   */
  virtual void execute(class Walker* context) = 0;
};

/**
 * @brief ForwardState implementation: Moves the robot forward.
 */
class ForwardState : public State {
 public:
  void execute(Walker* context) override;
};

/**
 * @brief ClockwiseState implementation: Turns the robot clockwise until clear.
 */
class ClockwiseState : public State {
 public:
  void execute(Walker* context) override;
};

/**
 * @brief AnticlockwiseState: Turns the robot counterclockwise until clear.
 */
class AnticlockwiseState : public State {
 public:
  void execute(Walker* context) override;
};

/**
 * @brief Walker class: ROS 2 Node managing the State Design Pattern.
 */
class Walker : public rclcpp::Node {
 public:
  Walker();

  void publish_velocity(const geometry_msgs::msg::Twist &cmd);
  bool is_obstacle_detected() const;
  void set_state(std::shared_ptr<State> state);
  rclcpp::Logger get_logger() const;

  bool last_turn_clockwise;  // Declaration order: 1st

 private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void execute();

  std::shared_ptr<State> current_state_;  // Declaration order: 2nd
  bool obstacle_detected_;  // Declaration order: 3rd

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};


#endif  // SRC_WALKER_INCLUDE_WALKER_HPP_
