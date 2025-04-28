#ifndef DDSM115_COMMUNICATOR_H
#define DDSM115_COMMUNICATOR_H

#include <pthread.h>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cstddef>

namespace ddsm115 {

static constexpr size_t PACKET_SIZE = 10;
static constexpr unsigned int BAUD_RATE = B115200;
static constexpr uint8_t CRC_POLY = 0x8C;

// Communication states
enum class State : uint8_t { NORMAL = 0x01, FAILED = 0x02 };
// Control modes
enum class Mode : uint8_t { CURRENT_LOOP = 0x01, VELOCITY_LOOP = 0x02, POSITION_LOOP = 0x03 };
// Command identifiers
enum class Command : uint8_t { DRIVE_MOTOR = 0x64, GET_FEEDBACK = 0x74, SWITCH_MODE = 0xA0, SET_ID = 0x55, QUERY_ID = 0x64 };

// Parsed feedback structure
struct Feedback {
  Mode    mode;
  double  current;  // A
  double  velocity; // rpm
  double  position; // degrees
  uint8_t error;
  State   status;
};

class Communicator {
public:
  explicit Communicator(const std::string &port_name);
  ~Communicator();

  void disconnect();
  State getState() const;

  void switchMode(uint8_t id, Mode m);
  void setID(uint8_t old_id, uint8_t new_id);
  uint8_t queryID(uint8_t id);
  Feedback driveMotor(uint8_t id, int16_t val);
  Feedback getFeedback(uint8_t id);

private:
  int port_fd_;
  State state_;
  pthread_mutex_t mutex_;

  void configurePort();
  void lock();
  void unlock();
  uint8_t computeCRC(const uint8_t *data, size_t len) const;
  void sendPacket(const uint8_t *pkt);
  void readPacket(uint8_t *buf);
  Feedback parseFeedback(const uint8_t *buf) const;
};

} // namespace ddsm115

#endif // DDSM115_COMMUNICATOR_H