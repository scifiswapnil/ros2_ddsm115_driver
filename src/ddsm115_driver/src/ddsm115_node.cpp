#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "ddsm115_driver/DDSM115Communicator.h"
#include "ddsm115_driver/srv/set_motor_id.hpp"
#include "ddsm115_driver/srv/query_motor_id.hpp"
#include "ddsm115_driver/srv/change_motor_mode.hpp"
#include <algorithm>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

struct MotorConfig {
  std::string name;
  uint8_t id;
  ddsm115::Mode mode;
};

class DDSM115Node : public rclcpp::Node {
public:
  DDSM115Node()
  : Node(declare_node_name_(), rclcpp::NodeOptions()
         .allow_undeclared_parameters(true))
  {
    // Callback groups for concurrency
    control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Declare and retrieve parameters
    declare_parameter("port_name", std::string("/dev/ttyUSB0"));
    declare_parameter("node_frequency", 20.0);
    declare_parameter("config_file", std::string("motors.yaml"));
    get_parameter("port_name", port_name_);
    get_parameter("node_frequency", freq_);
    get_parameter("config_file", config_file_);

    // Initialize communicator
    comm_ = std::make_unique<ddsm115::Communicator>(port_name_);

    // Load motor configurations
    YAML::Node cfg;
    try {
      cfg = YAML::LoadFile(config_file_);
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to load config '%s': %s", config_file_.c_str(), e.what());
      rclcpp::shutdown();
      return;
    }
    auto joints = cfg["joints"];
    if (!joints || !joints.IsSequence()) {
      RCLCPP_ERROR(get_logger(), "Config must contain a 'joints' sequence");
      rclcpp::shutdown();
      return;
    }

    motors_.reserve(joints.size());
    for (auto m : joints) {
      MotorConfig mc;
      mc.name = m["name"].as<std::string>();
      mc.id   = static_cast<uint8_t>(m["id"].as<int>());
      auto mode_str = m["mode"].as<std::string>();
      if (mode_str == "position") mc.mode = ddsm115::Mode::POSITION_LOOP;
      else if (mode_str == "velocity") mc.mode = ddsm115::Mode::VELOCITY_LOOP;
      else if (mode_str == "current")  mc.mode = ddsm115::Mode::CURRENT_LOOP;
      else {
        RCLCPP_WARN(get_logger(), "Unknown mode '%s' -> default to position", mode_str.c_str());
        mc.mode = ddsm115::Mode::POSITION_LOOP;
      }
      motors_.push_back(mc);
    }

    if (comm_->getState() != ddsm115::State::NORMAL) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port '%s'", port_name_.c_str());
      rclcpp::shutdown();
      return;
    }

    // Set each motor's mode
    for (auto &mc : motors_) {
      comm_->switchMode(mc.id, mc.mode);
    }

    // Create subscriptions for targets
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = control_cb_group_;
    for (auto &mc : motors_) {
      subs_.push_back(
        this->create_subscription<std_msgs::msg::Float64>(
          mc.name + "/target", 10,
          [this, id = mc.id](const std_msgs::msg::Float64::SharedPtr msg) {
            handle_target(id, msg->data);
          }, sub_opts));
    }

    // Create services
    auto qos = rmw_qos_profile_services_default;
    set_id_srv_ = this->create_service<ddsm115_driver::srv::SetMotorId>(
      "set_motor_id",
      [this](const std::shared_ptr<ddsm115_driver::srv::SetMotorId::Request> req,
             std::shared_ptr<ddsm115_driver::srv::SetMotorId::Response> res) {
        for (int i = 0; i < 5; ++i) comm_->setID(req->id);
        res->success = (comm_->queryID().id == req->id);
      }, qos, service_cb_group_);

    query_id_srv_ = this->create_service<ddsm115_driver::srv::QueryMotorId>(
      "query_motor_id",
      [this](const std::shared_ptr<ddsm115_driver::srv::QueryMotorId::Request>,
             std::shared_ptr<ddsm115_driver::srv::QueryMotorId::Response> res) {
        res->id = comm_->queryID().id;
      }, qos, service_cb_group_);

    change_mode_srv_ = this->create_service<ddsm115_driver::srv::ChangeMotorMode>(
      "change_motor_mode",
      [this](const std::shared_ptr<ddsm115_driver::srv::ChangeMotorMode::Request> req,
             std::shared_ptr<ddsm115_driver::srv::ChangeMotorMode::Response> res) {
        comm_->switchMode(req->id, static_cast<ddsm115::Mode>(req->mode));
        auto fb = comm_->getAdditionalFeedback(req->id);
        res->success = (fb.mode == static_cast<ddsm115::Mode>(req->mode));
        for (auto &mc : motors_) {
          if (mc.id == req->id) { mc.mode = static_cast<ddsm115::Mode>(req->mode); break; }
        }

        if (res->success)
          RCLCPP_INFO(get_logger(), "Motor %u mode switched successfully", req->id);
        else
          RCLCPP_WARN(get_logger(), "Motor %u mode switch failed: got mode %u", req->id, static_cast<uint8_t>(fb.mode));
      }, qos, service_cb_group_);

    // Publishers for feedback
    for (auto &mc : motors_) {
      torque_pubs_.push_back(this->create_publisher<std_msgs::msg::Float64>(mc.name + "/feedback/torque", 10));
      velocity_pubs_.push_back(this->create_publisher<std_msgs::msg::Float64>(mc.name + "/feedback/velocity", 10));
      position_pubs_.push_back(this->create_publisher<std_msgs::msg::Float64>(mc.name + "/feedback/position", 10));
      temperature_pubs_.push_back(this->create_publisher<std_msgs::msg::Float64>(mc.name + "/feedback/temperature", 10));
      error_pubs_.push_back(this->create_publisher<std_msgs::msg::UInt8>(mc.name + "/feedback/error_code", 10));
    }

    // Timer for periodic feedback
    feedback_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / freq_),
      [this]() { publish_feedback(); },
      control_cb_group_);
  }

private:
  void handle_target(uint8_t id, double input) {
    auto it = std::find_if(motors_.begin(), motors_.end(),
                           [id](const MotorConfig &m) { return m.id == id; });
    auto mode = (it != motors_.end()) ? it->mode : ddsm115::Mode::POSITION_LOOP;
    int16_t raw = 0;
    switch (mode) {
      case ddsm115::Mode::CURRENT_LOOP: {
        double scaled = input * (32767.0 / 8.0);
        raw = static_cast<int16_t>(std::clamp(scaled, -32767.0, 32767.0));
        // std::cout << "Current: " << raw << std::endl;
        break;
      }
      case ddsm115::Mode::VELOCITY_LOOP: {
        double clamped = std::clamp(input, -330.0, 330.0);
        raw = static_cast<int16_t>(clamped);
        // std::cout << "Velocity: " << raw << std::endl;
        break;
      }
      case ddsm115::Mode::POSITION_LOOP: {
        double angle = input < 0.0 ? 360.0 + input : input;
        angle = std::fmod(angle, 360.0);
        double scaled = angle * (32767.0 / 360.0);
        raw = static_cast<int16_t>(std::clamp(scaled, 0.0, 32767.0));
        // std::cout << "Position: " << raw << std::endl;
        break;
      }
    }
    comm_->driveMotor(id, raw, 0, 0);
  }

  void publish_feedback() {
    for (size_t i = 0; i < motors_.size(); ++i) {
      auto fb = comm_->getAdditionalFeedback(motors_[i].id);
      std_msgs::msg::Float64 msg;
      msg.data = fb.current;
      torque_pubs_[i]->publish(msg);
      msg.data = fb.velocity;
      velocity_pubs_[i]->publish(msg);
      msg.data = fb.position ;
      position_pubs_[i]->publish(msg);
      msg.data = 0.0;
      temperature_pubs_[i]->publish(msg);
      std_msgs::msg::UInt8 err;
      err.data = fb.error;
      error_pubs_[i]->publish(err);
    }
  }

  static std::string declare_node_name_() { return "ddsm115_node"; }
  std::string port_name_, config_file_;
  double freq_;
  std::unique_ptr<ddsm115::Communicator> comm_;
  std::vector<MotorConfig> motors_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subs_;
  rclcpp::CallbackGroup::SharedPtr control_cb_group_, service_cb_group_;
  rclcpp::Service<ddsm115_driver::srv::SetMotorId>::SharedPtr set_id_srv_;
  rclcpp::Service<ddsm115_driver::srv::QueryMotorId>::SharedPtr query_id_srv_;
  rclcpp::Service<ddsm115_driver::srv::ChangeMotorMode>::SharedPtr change_mode_srv_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> torque_pubs_, velocity_pubs_, position_pubs_, temperature_pubs_;
  std::vector<rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr> error_pubs_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DDSM115Node>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <yaml-cpp/yaml.h>
// #include <std_msgs/msg/float64.hpp>
// #include <std_msgs/msg/u_int8.hpp>
// #include "ddsm115_driver/DDSM115Communicator.h"
// #include "ddsm115_driver/srv/set_motor_id.hpp"
// #include "ddsm115_driver/srv/query_motor_id.hpp"
// #include "ddsm115_driver/srv/change_motor_mode.hpp"

// using namespace std::chrono_literals;

// struct MotorConfig {
//   std::string name;
//   uint8_t id;
//   ddsm115::Mode mode;
// };

// class DDSM115Node : public rclcpp::Node {
// public:
//   DDSM115Node()
//   : Node(declare_node_name_(), rclcpp::NodeOptions()
//       .allow_undeclared_parameters(true)
//       .automatically_declare_parameters_from_overrides(true)),
//     comm_(get_parameter("port_name").as_string())
//   {


//     service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


//     // // Read parameters
//     port_name_ = get_parameter("port_name").as_string();
//     freq_      = get_parameter("node_frequency").as_double();
//     config_file_ = get_parameter("config_file").as_string();
//     // declare_parameter("port_name", std::string("/dev/ttyUSB0"));
//     // declare_parameter("node_frequency", 20.0);
//     // declare_parameter("config_file", std::string("motors.yaml"));
//     // port_name_   = get_parameter("port_name").as_string();
//     // freq_        = get_parameter("node_frequency").as_double();
//     // config_file_ = get_parameter("config_file").as_string();

//     // Load motor configurations
//     YAML::Node cfg = YAML::LoadFile(config_file_);
//     motors_.reserve(cfg["joints"].size());
//     for (auto m : cfg["joints"]) {
//       MotorConfig mc;
//       mc.name = m["name"].as<std::string>();
//       mc.id   = static_cast<uint8_t>(m["id"].as<int>());
//       std::string mode = m["mode"].as<std::string>();
//       if (mode == "position" || mode == "POSITION_LOOP") mc.mode = ddsm115::Mode::POSITION_LOOP;
//       else if (mode == "velocity" || mode == "VELOCITY_LOOP") mc.mode = ddsm115::Mode::VELOCITY_LOOP;
//       else if (mode == "current"  || mode == "CURRENT_LOOP")  mc.mode = ddsm115::Mode::CURRENT_LOOP;
//       else RCLCPP_WARN(get_logger(), "Unknown mode '%s' for joint '%s'", mode.c_str(), mc.name.c_str());
//       motors_.push_back(mc);
//     }

//     // Initialize communicator
//     if (comm_.getState() != ddsm115::State::NORMAL) {
//       RCLCPP_ERROR(get_logger(), "Failed to open serial port '%s'", port_name_.c_str());
//       rclcpp::shutdown();
//       return;
//     }

//     // Switch each motor to its mode
//     for (auto &mc : motors_) {
//       comm_.switchMode(mc.id, mc.mode);
//     }

//     // Subscribers for each motor
//     for (auto &mc : motors_) {
//       std::string topic = mc.name + "/target";
//       subs_.push_back(
//         create_subscription<std_msgs::msg::Float64>(
//           topic, 10,
//           [this, id = mc.id](const std_msgs::msg::Float64::SharedPtr msg) {
//             // Find the up-to-date mode for this motor
//             auto it = std::find_if(
//               motors_.begin(), motors_.end(),
//               [id](const MotorConfig &m){ return m.id == id; }
//             );
//             if (it == motors_.end()) {
//               RCLCPP_WARN(get_logger(), "No config for motor ID %u", id);
//               return;
//             }
//             ddsm115::Mode mode = it->mode;
//             double in = msg->data;
//             int16_t raw = 0;
//             switch (mode) {
//               case ddsm115::Mode::CURRENT_LOOP: {
//                 double scaled = in * (32767.0 / 8.0);
//                 raw = static_cast<int16_t>(std::clamp(scaled, -32767.0, 32767.0));
//                 break;
//               }
//               case ddsm115::Mode::VELOCITY_LOOP: {
//                 double clamped = std::clamp(in, -330.0, 330.0);
//                 raw = static_cast<int16_t>(clamped);
//                 break;
//               }
//               case ddsm115::Mode::POSITION_LOOP: {
//                 double angle = in;
//                 if (angle < 0.0) angle = 360.0 + angle;
//                 angle = fmod(angle, 360.0);
//                 double scaled = angle * (32767.0 / 360.0);
//                 raw = static_cast<int16_t>(std::clamp(scaled, 0.0, 32767.0));
//                 break;
//               }
//             }
//             comm_.driveMotor(id, raw, 0, 0);
//           }
//         )
//       );
//     }

//     rclcpp::SubscriptionOptions sub_options;
//     sub_options.callback_group = control_cb_group_;
//     for (auto &mc : motors_) {
//       auto topic = mc.name + "/target";
//       subs_.push_back(create_subscription<std_msgs::msg::Float64>(
//         topic, 10,
//         [this, id = mc.id](const std_msgs::msg::Float64::SharedPtr msg) {
//           handle_target(id, msg->data);
//         }, sub_options));
//     }

//     for (auto &mc : motors_) {
//       torque_pubs_.push_back(create_publisher<std_msgs::msg::Float64>(mc.name + "/feedback/torque", 10));
//       velocity_pubs_.push_back(create_publisher<std_msgs::msg::Float64>(mc.name + "/feedback/velocity", 10));
//       position_pubs_.push_back(create_publisher<std_msgs::msg::Float64>(mc.name + "/feedback/position", 10));
//       temperature_pubs_.push_back(create_publisher<std_msgs::msg::Float64>(mc.name + "/feedback/temperature", 10));
//       error_pubs_.push_back(create_publisher<std_msgs::msg::UInt8>(mc.name + "/feedback/error_code", 10));
//     }

//     // Poll feedback at the configured frequency
//     feedback_timer_ = this->create_wall_timer(
//       std::chrono::duration<double>(1.0/freq_),
//       [this]() { publish_feedback(); },
//       control_cb_group_);

//     // Service: Set Motor ID
//     set_id_srv_ = create_service<ddsm115_driver::srv::SetMotorId>(
//       "set_motor_id",
//       [this](const std::shared_ptr<ddsm115_driver::srv::SetMotorId::Request> req,
//              std::shared_ptr<ddsm115_driver::srv::SetMotorId::Response> res) {
//         uint8_t new_id = static_cast<uint8_t>(req->id);
//         for (int i = 0; i < 5; ++i) comm_.setID(new_id);
//         auto fb = comm_.queryID();
//         res->success = (fb.id == new_id);
//       }
//     );

//     // Service: Query Motor ID
//     query_id_srv_ = create_service<ddsm115_driver::srv::QueryMotorId>(
//       "query_motor_id",
//       [this](const std::shared_ptr<ddsm115_driver::srv::QueryMotorId::Request> /*req*/,
//              std::shared_ptr<ddsm115_driver::srv::QueryMotorId::Response> res) {
//         auto fb = comm_.queryID();
//         res->id = fb.id;
//       }
//     );

//     change_mode_srv_ = create_service<ddsm115_driver::srv::ChangeMotorMode>(
//       "change_motor_mode",
//       [this](const std::shared_ptr<ddsm115_driver::srv::ChangeMotorMode::Request> req, 
//            std::shared_ptr<ddsm115_driver::srv::ChangeMotorMode::Response> res)  {
//             uint8_t id   = static_cast<uint8_t>(req->id);
//             auto mode    = static_cast<ddsm115::Mode>(req->mode);
//             comm_.switchMode(id, mode);
//             for (auto &mc : motors_) {
//               if (mc.id == id) { mc.mode = mode; break; }
//             }
//             res->success = true;
//             RCLCPP_INFO(get_logger(), "Changed motor %u to mode %u", id, req->mode);
//           }
//     );

//     // Timer to keep node alive at specified frequency
//     timer_ = create_wall_timer(
//       std::chrono::duration<double>(1.0 / freq_),
//       []() {}
//     );
//   }

// private:
// void publish_feedback() {
//   for (size_t i = 0; i < motors_.size(); ++i) {
//     auto fb = comm_.getAdditionalFeedback(motors_[i].id);
//     std_msgs::msg::Float64 fm;
//     fm.data = fb.current; torque_pubs_[i]->publish(fm);
//     fm.data = fb.velocity; velocity_pubs_[i]->publish(fm);
//     fm.data = (fb.position/255.0)*360.0; position_pubs_[i]->publish(fm);
//     fm.data = 0.0; temperature_pubs_[i]->publish(fm);
//     std_msgs::msg::UInt8 em; em.data = fb.error; error_pubs_[i]->publish(em);
//   }
// }

//   void handle_target(uint8_t id, double in) {
//   auto it = std::find_if(motors_.begin(), motors_.end(), [id](auto &m){ return m.id == id; });
//   auto mode = (it != motors_.end()) ? it->mode : ddsm115::Mode::POSITION_LOOP;
//   int16_t raw = 0;
//   switch (mode) {
//     case ddsm115::Mode::CURRENT_LOOP:
//       raw = static_cast<int16_t>(std::clamp(in * (32767.0/8.0), -32767.0, 32767.0));
//       break;
//     case ddsm115::Mode::VELOCITY_LOOP:
//       raw = static_cast<int16_t>(std::clamp(in, -330.0, 330.0));
//       break;
//     case ddsm115::Mode::POSITION_LOOP: {
//       double angle = in < 0.0 ? 360.0 + in : in;
//       angle = fmod(angle, 360.0);
//       raw = static_cast<int16_t>(std::clamp(angle * (32767.0/360.0), 0.0, 32767.0));
//       break;
//     }
//   }
//   comm_.driveMotor(id, raw, 0, 0);
// }

//   static std::string declare_node_name_() {
//     // Declare node name parameter default
//     return "ddsm115_node";
//   }

//   std::string port_name_, config_file_;
//   double      freq_;
//   ddsm115::Communicator comm_;
//   std::vector<MotorConfig> motors_;
//   std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subs_;
//   rclcpp::Service<ddsm115_driver::srv::SetMotorId>::SharedPtr    set_id_srv_;
//   rclcpp::Service<ddsm115_driver::srv::QueryMotorId>::SharedPtr  query_id_srv_;
//   rclcpp::Service<ddsm115_driver::srv::ChangeMotorMode>::SharedPtr change_mode_srv_;
//   std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> torque_pubs_, velocity_pubs_, position_pubs_, temperature_pubs_;
//   std::vector<rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr> error_pubs_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::TimerBase::SharedPtr feedback_timer_;
//   rclcpp::CallbackGroup::SharedPtr control_cb_group_, service_cb_group_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<DDSM115Node>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

// // Sample config file (motors.yaml):
// // joints:
// //   - name: joint1
// //     id: 1
// //     mode: position
// //   - name: joint2
// //     id: 2
// //     mode: velocity
// //   - name: joint3
// //     id: 3
// //     mode: current
