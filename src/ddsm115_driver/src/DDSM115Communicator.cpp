// Implementation (.cpp)
#include "ddsm115_driver/DDSM115Communicator.h"
#include <sstream>
#include <iomanip>

namespace ddsm115 {

static const auto LOGGER = rclcpp::get_logger("DDSM115Communicator");

Communicator::Communicator(const std::string &port_name)
 : port_fd_(-1), state_(State::FAILED)
{
  port_fd_ = open(port_name.c_str(), O_RDWR);
  if (port_fd_ < 0) {
    RCLCPP_ERROR(LOGGER, "Failed to open %s: %s", port_name.c_str(), strerror(errno));
    return;
  }
  configurePort();
  state_ = State::NORMAL;
}

Communicator::~Communicator() {
  disconnect();
}

void Communicator::disconnect() {
  if (port_fd_ >= 0) close(port_fd_);
  port_fd_ = -1;
}

State Communicator::getState() const {
  return state_;
}

void Communicator::configurePort() {
  struct termios tty{};
  if (tcgetattr(port_fd_, &tty) != 0) {
    RCLCPP_ERROR(LOGGER, "tcgetattr failed: %s", strerror(errno));
    state_ = State::FAILED;
    return;
  }
  tty.c_cflag = CS8 | CLOCAL | CREAD;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN]  = 0;
  cfsetispeed(&tty, BAUD_RATE);
  cfsetospeed(&tty, BAUD_RATE);
  if (tcsetattr(port_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(LOGGER, "tcsetattr failed: %s", strerror(errno));
    state_ = State::FAILED;
  }
}

uint8_t Communicator::computeCRC(const uint8_t *data, size_t len) const {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    uint8_t in = data[i];
    for (int b = 0; b < 8; ++b) {
      bool mix = (crc ^ in) & 0x01;
      crc >>= 1;
      if (mix) crc ^= CRC_POLY;
      in >>= 1;
    }
  }
  return crc;
}

void Communicator::sendPacket(const uint8_t *pkt) {
  std::lock_guard<std::mutex> guard(mutex_);
  ssize_t written = write(port_fd_, pkt, PACKET_SIZE);
  if (written != PACKET_SIZE) {
    RCLCPP_ERROR(LOGGER, "sendPacket: expected %zu but wrote %zd: %s", PACKET_SIZE, written, strerror(errno));
  } else {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < PACKET_SIZE; ++i) ss << std::setw(2) << int(pkt[i]) << ' ';
    // RCLCPP_INFO(LOGGER, "Sent packet: %s", ss.str().c_str());
  }
}

void Communicator::readPacket(uint8_t *buf) {
  int total = 0, rc;
  while (total < PACKET_SIZE) {
    // std::cout<<"Reading packet: " << total << std::endl;
    rc = read(port_fd_, buf + total, PACKET_SIZE - total);
    if (rc <= 0) break;
    total += rc;
  }
  if (total > 0) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (int i = 0; i < total; ++i) ss << std::setw(2) << int(buf[i]) << ' ';
    // RCLCPP_INFO(LOGGER, "Received %d bytes: %s", total, ss.str().c_str());
  } else {
    RCLCPP_WARN(LOGGER, "readPacket: no data received");
  }
}

void Communicator::switchMode(uint8_t id, Mode m) {
  uint8_t pkt[PACKET_SIZE] = {id,static_cast<uint8_t>(Command::SWITCH_MODE)};
  memset(pkt + 2, 0, PACKET_SIZE - 3);
  pkt[8] = 0;
  pkt[9] = static_cast<uint8_t>(m); //computeCRC(pkt, 9);
  sendPacket(pkt);
}

void Communicator::setID(uint8_t new_id) {
  uint8_t pkt[PACKET_SIZE] = { 0xAA, static_cast<uint8_t>(Command::SET_ID), 0x53, new_id };
  memset(pkt + 4, 0, PACKET_SIZE - 5);
  pkt[9] = computeCRC(pkt, 9);
  sendPacket(pkt);
}

Feedback Communicator::queryID() {
  // Packet: DATA[0]=0xC8, DATA[1]=0x64
  uint8_t pkt[PACKET_SIZE] = { 0xC8, 0x64 };
  memset(pkt + 2, 0, PACKET_SIZE - 3);
  pkt[9] = computeCRC(pkt, 9);
  sendPacket(pkt);

  uint8_t buf[PACKET_SIZE] = {0};
  readPacket(buf);
  return parseFeedback(buf);
}

// Updated driveMotor: include acceleration time and brake
Feedback Communicator::driveMotor(uint8_t id, int16_t value, uint8_t acc_time, uint8_t brake) {
  uint8_t pkt[PACKET_SIZE] = {
    id,
    static_cast<uint8_t>(Command::DRIVE_MOTOR),
    static_cast<uint8_t>(value >> 8),
    static_cast<uint8_t>(value & 0xFF),
    0,
    0,
    acc_time,
    brake,
    0
  };
  pkt[9] = computeCRC(pkt, 9);
  sendPacket(pkt);
  uint8_t buf[PACKET_SIZE] = {0};
  readPacket(buf);
  return parseFeedback(buf);
}

Feedback Communicator::parseFeedback(const uint8_t *buf) const {
  Feedback fb;
  fb.status           = (buf[9] == computeCRC(buf, 9)) ? State::NORMAL : State::FAILED;
  fb.id               = buf[0];
  fb.mode             = static_cast<Mode>(buf[1]);
  int16_t tc          = (buf[2] << 8) | buf[3];
  int16_t tv          = (buf[4] << 8) | buf[5];
  int16_t tp          = (buf[6] << 8) | buf[7];
  fb.current          = double(tc) * (8.0 / 32767.0);
  fb.velocity         = double(tv) ;//* ( 32767.0);
  fb.position         = double(tp) * (360.0 / 32767.0);     // Raw units
  if (fb.position < 0) fb.position += 360.0; // Wrap into [0,360) 
  fb.error            = buf[8];
  return fb;
}

Feedback Communicator::parseAdditionalFeedback(const uint8_t *buf) const {
  Feedback fb;
  fb.status           = (buf[9] == computeCRC(buf, 9)) ? State::NORMAL : State::FAILED;
  fb.id               = buf[0];
  fb.mode             = static_cast<Mode>(buf[1]);
  int16_t tc          = (buf[2] << 8) | buf[3];
  int16_t tv          = (buf[4] << 8) | buf[5];
  fb.temperature      = buf[6];
  fb.error_code       = buf[8];
  fb.current          = double(tc) * (8.0 / 32767.0);
  fb.velocity         = double(tv) ;//* (32767.0);
  fb.position         = (static_cast<int>(buf[7]) / 255.0) * 360.0;;
  if (fb.position < 0) fb.position += 360.0; // Wrap into [0,360) 
  fb.error            = buf[8];
  return fb;
}

Feedback Communicator::getAdditionalFeedback(uint8_t id) {
  uint8_t pkt[PACKET_SIZE] = {id, static_cast<uint8_t>(Command::GET_FEEDBACK)};
  memset(pkt + 2, 0, PACKET_SIZE - 3);
  pkt[9] = computeCRC(pkt, 9);
  sendPacket(pkt);
  uint8_t buf[PACKET_SIZE] = {0};
  readPacket(buf);
  return parseAdditionalFeedback(buf);
}

} // namespace ddsm115
