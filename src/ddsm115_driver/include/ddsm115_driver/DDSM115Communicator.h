#ifndef DDSM115_COMMUNICATOR_H
#define DDSM115_COMMUNICATOR_H

// Updated for ROS2: use rclcpp for logging
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>

namespace ddsm115 {

static constexpr size_t PACKET_SIZE = 10;
static constexpr unsigned int BAUD_RATE = B115200;
static constexpr uint8_t CRC_POLY = 0x8C;

enum class State : uint8_t { NORMAL = 0x01, FAILED = 0x02 };
enum class Mode  : uint8_t { CURRENT_LOOP = 0x01, VELOCITY_LOOP = 0x02, POSITION_LOOP = 0x03 };
enum class ErrorCode : uint8_t {
  SENSOR_ERROR            = 0x01,
  OVERCURRENT_ERROR       = 0x02,
  PHASE_OVERCURRENT_ERROR = 0x04,
  STALL_ERROR             = 0x08,
  TROUBLESHOOTING_ERROR   = 0x10,
};
enum class Command : uint8_t { DRIVE_MOTOR = 0x64, GET_FEEDBACK = 0x74, SWITCH_MODE = 0xA0, SET_ID = 0x55, QUERY_ID = 0x64 };

struct Feedback {
  Mode    mode;
  uint8_t id;
  double  current;
  double  velocity;
  double  position;
  uint8_t error;
  State   status;
};

class Communicator {
public:
  explicit Communicator(const std::string &port_name);
  ~Communicator();

  void    disconnect();
  State   getState() const;

  void     switchMode(uint8_t id, Mode m);
  void     setID(uint8_t new_id);
  Feedback  queryID();

  Feedback driveMotor(uint8_t id, int16_t value, uint8_t acc_time, uint8_t brake);
  Feedback getAdditionalFeedback(uint8_t id);

private:
  int            port_fd_;
  State          state_;
  std::mutex     mutex_;

  void configurePort();
  uint8_t computeCRC(const uint8_t *data, size_t len) const;
  void sendPacket(const uint8_t *packet);
  void readPacket(uint8_t *buffer);
  Feedback parseFeedback(const uint8_t *buf) const;
};

} // namespace ddsm115
#endif // DDSM115_COMMUNICATOR_H
