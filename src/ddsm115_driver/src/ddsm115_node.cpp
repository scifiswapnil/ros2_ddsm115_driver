// ddsm115_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include "ddsm115_driver/DDSM115Communicator.h"

using namespace ddsm115;

struct MotorConfig {
  uint8_t id;
  Mode mode;
};

class Ddsm115Node : public rclcpp::Node {
public:
  Ddsm115Node()
  : Node("ddsm115_node")
  {
    // Load parameters from config.yaml
    this->declare_parameter("config_file", "config/config.yaml");
    std::string config_file;
    this->get_parameter("config_file", config_file);

    YAML::Node cfg = YAML::LoadFile(config_file);
    serial_port_ = cfg["serial_port"].as<std::string>();
    publish_rate_ = cfg["publish_rate"].as<int>();

    for (auto it : cfg["motors"]) {
      std::string joint = it.first.as<std::string>();
      auto mnode = it.second;
      MotorConfig mc;
      mc.id = mnode["id"].as<int>();
      std::string m = mnode["mode"].as<std::string>();
      if (m == "current") mc.mode = Mode::CURRENT_LOOP;
      else if (m == "position") {mc.mode = Mode::POSITION_LOOP; std::cout << "position mode" << std::endl;} 
      else mc.mode = Mode::VELOCITY_LOOP;
      motors_[joint] = mc;
    }

    // Initialize communicator
    comm_ = std::make_shared<Communicator>(serial_port_);
    if (comm_->getState() != State::NORMAL) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port %s", serial_port_.c_str());
      rclcpp::shutdown();
    }

    // Set modes
    for (auto &p : motors_) {
      comm_->switchMode(p.second.id, p.second.mode);
    }

    // Publisher
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Subscribers: store last commands
    for (auto &p : motors_) {
      auto joint = p.first;
      auto id    = p.second.id;

      auto vel_topic = joint + "/cmd_vel";
      auto pos_topic = joint + "/cmd_pos";

      auto vel_cb = [this, joint](std_msgs::msg::Float64::SharedPtr msg) {
        // store raw velocity command
        last_vel_cmd_[joint] = static_cast<int16_t>(msg->data);
      };
      auto pos_cb = [this, joint](std_msgs::msg::Float64::SharedPtr msg) {
        // convert radian to raw encoder units
        double deg = msg->data * (180.0/M_PI);
        last_pos_cmd_[joint] = static_cast<int16_t>((deg / 360.0) * 32767);
        std::cout << "pos cmd: " << last_pos_cmd_[joint] << std::endl;
      };

      subs_.push_back(
        this->create_subscription<std_msgs::msg::Float64>(vel_topic, 10, vel_cb));
      subs_.push_back(
        this->create_subscription<std_msgs::msg::Float64>(pos_topic, 10, pos_cb));
    }

    // Timer for publishing feedback (and re-sending last command)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(int(1000.0/publish_rate_)),
      std::bind(&Ddsm115Node::publishFeedback, this));
  }

private:
  void publishFeedback() {
    // Re-send last commands to hold setpoints
    for (auto &p : motors_) {
      const auto &joint = p.first;
      const auto &mc    = p.second;
      auto id = mc.id;
      if (mc.mode == Mode::POSITION_LOOP) {
        auto it = last_pos_cmd_.find(joint);
        if (it != last_pos_cmd_.end()) {
          comm_->driveMotor(id, it->second);
        }
      } else if (mc.mode == Mode::VELOCITY_LOOP) {
        auto it = last_vel_cmd_.find(joint);
        if (it != last_vel_cmd_.end()) {
          comm_->driveMotor(id, it->second);
        }
      }
      // For current loop, implement similarly if needed
    }

    // Gather feedback and publish
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    for (auto &p : motors_) {
      const auto &joint = p.first;
      const auto &mc    = p.second;
      auto fb = comm_->getFeedback(mc.id);
      msg.name.push_back(joint);
      msg.position.push_back(fb.position * M_PI/180.0);
      msg.velocity.push_back(fb.velocity * M_PI/30.0);
      msg.effort.push_back(fb.current);
    }
    joint_state_pub_->publish(msg);
  }

  std::string serial_port_;
  int publish_rate_;
  std::shared_ptr<Communicator> comm_;
  std::map<std::string, MotorConfig> motors_;
  std::map<std::string, int16_t> last_pos_cmd_;
  std::map<std::string, int16_t> last_vel_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ddsm115Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
