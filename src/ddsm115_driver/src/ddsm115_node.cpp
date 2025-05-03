// motor_test_position_node.cpp
// ROS2 node to test ddsm115::Communicator in Position Loop mode with target reach detection

#include <rclcpp/rclcpp.hpp>
#include "ddsm115_driver/DDSM115Communicator.h"

using namespace std::chrono_literals;
using ddsm115::Communicator;
using ddsm115::Mode;
using ddsm115::State;

uint16_t angleToMotorCount(double angleDeg) {
  // Scale factor: counts per degree
  constexpr double COUNTS_PER_DEG = 32767.0 / 360.0;

  // Wrap into [0,360):
  double wrapped = std::fmod(angleDeg, 360.0);
  if (wrapped < 0) wrapped += 360.0;

  // Convert to counts (round nearest):
  double rawCount = wrapped * COUNTS_PER_DEG;
  uint16_t count = static_cast<uint16_t>(std::round(rawCount));

  // Safety clamp (in case of rounding up to 32768)
  return (count > 32767 ? 32767 : count);
}

class MotorPositionTestNode : public rclcpp::Node {
public:
  MotorPositionTestNode()
  : Node("motor_position_test_node"),
    comm_("/dev/ttyUSB0"),
    forward_(true),
    target_deg_(0.0),
    reached_(true)
  {
    // if (comm_.getState() != State::NORMAL) {
    //   RCLCPP_FATAL(get_logger(), "Communicator init failed on %s", "/dev/ttyACM0");
    //   rclcpp::shutdown();
    //   return;
    // }
    
    // for (int i = 0; i < 5; ++i) {
    //   comm_.setID(10);
    // }

    auto fb =  comm_.queryID();
    RCLCPP_INFO(get_logger(), "Motor ID: %d", fb.id);
    // // Switch to POSITION_LOOP mode
    // comm_.switchMode(1, Mode::POSITION_LOOP);
    // RCLCPP_INFO(get_logger(), "Motor ID 1 set to POSITION_LOOP mode");

    // // Timer: toggle command every 5 seconds
    // command_timer_ = create_wall_timer(
    //   150ms, std::bind(&MotorPositionTestNode::onCommandTimer, this)
    // );

    // Timer: poll feedback every 500ms
    // feedback_timer_ = create_wall_timer(
    //   5s, std::bind(&MotorPositionTestNode::onFeedbackTimer, this)
    // );
  }

private:
  void onCommandTimer() {
    // Only issue new command if previous target reached
    // if (!reached_) return;

    // Determine next target angle
    target_deg_ = forward_ ?  45.0 : -45.0;
    // Convert degrees to raw units
    int16_t raw_target = angleToMotorCount(target_deg_);//static_cast<int16_t>(target_deg_ * 32767.0 / 360.0);

    // auto fb = comm_.driveMotor(1, raw_target, Mode::VELOCITY_LOOP);
    auto fb = comm_.driveMotor(1, raw_target, 0.1, 1);
    // RCLCPP_INFO(get_logger(), "Commanded target: %.2f° (raw=%d)", target_deg_, raw_target);
    RCLCPP_DEBUG(
      get_logger(),
      "Monitor | pos: %.2f° | vel: %.2f rpm | cur: %.2f A | err: 0x%02X | status: %s",
      fb.position, fb.velocity, fb.current,
      fb.error,
      (fb.status == State::NORMAL ? "OK" : "CRC_FAIL")
    );

    if (std::abs(angleToMotorCount(fb.position) - angleToMotorCount(target_deg_)) < 10) {
      RCLCPP_INFO(get_logger(), "Reached target: %.2f°", target_deg_);
      forward_ = !forward_;
    }else{
      // Mark as awaiting reach
      
    }
  }

  void onFeedbackTimer() {
    // auto fb = comm_.getFeedback(1); //motor ID 1
    // RCLCPP_DEBUG(
    //   get_logger(),
    //   "Monitor | pos: %.2f° | vel: %.2f rpm | cur: %.2f A | err: 0x%02X | status: %s",
    //   fb.position, fb.velocity, fb.current,
    //   fb.error,
    //   (fb.status == State::NORMAL ? "OK" : "CRC_FAIL")
    // );
    // // Check if target reached within 1° tolerance
    // if (!reached_ && std::abs(fb.position - target_deg_) < 5.0) {
    //   RCLCPP_INFO(get_logger(), "Reached target: %.2f°", target_deg_);
    //   reached_ = true;
    // }
  }

  Communicator comm_;
  bool forward_;
  double target_deg_;    // Current target in degrees
  bool reached_;         // Flag whether target reached
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorPositionTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
