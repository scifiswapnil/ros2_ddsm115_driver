// --------------------------------------------------------
// Implementation (.cpp)
#include "ddsm115_driver/DDSM115Communicator.h"
#include <cstring>
#include <iostream>
#include <iomanip>
#include <unistd.h>

namespace ddsm115 {

Communicator::Communicator(const std::string &port_name)
  : port_fd_(-1), state_(State::FAILED) {
  pthread_mutex_init(&mutex_, nullptr);
  port_fd_ = open(port_name.c_str(), O_RDWR);
  if (port_fd_ < 0) {
    std::cerr << "Failed to open port " << port_name << ": " << strerror(errno) << std::endl;
    return;
  }
  configurePort();
  state_ = State::NORMAL;
}

Communicator::~Communicator() {
  disconnect();
  pthread_mutex_destroy(&mutex_);
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
    std::cerr << "tcgetattr failed: " << strerror(errno) << std::endl;
    state_ = State::FAILED;
    return;
  }
  tty.c_cflag = CS8 | CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | CSTOPB);
  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN]  = 0;
  cfsetispeed(&tty, BAUD_RATE);
  cfsetospeed(&tty, BAUD_RATE);
  if (tcsetattr(port_fd_, TCSANOW, &tty) != 0) {
    std::cerr << "tcsetattr failed: " << strerror(errno) << std::endl;
    state_ = State::FAILED;
  }
}

void Communicator::lock() {
  pthread_mutex_lock(&mutex_);
}

void Communicator::unlock() {
  pthread_mutex_unlock(&mutex_);
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
  lock();
  write(port_fd_, pkt, PACKET_SIZE);
  unlock();
}

void Communicator::readPacket(uint8_t *buf) {
  int total = 0;
  while (total < static_cast<int>(PACKET_SIZE)) {
    int rc = read(port_fd_, buf + total, PACKET_SIZE - total);
    if (rc <= 0) break;
    total += rc;
  }
}

Feedback Communicator::parseFeedback(const uint8_t *buf) const {
  Feedback fb;
  fb.status = (buf[9] == computeCRC(buf, 9)) ? State::NORMAL : State::FAILED;
  fb.mode   = static_cast<Mode>(buf[1]);

  int16_t t_cur = (buf[2] << 8) | buf[3];
  int16_t t_vel = (buf[4] << 8) | buf[5];
  fb.current  = double(t_cur) * (8.0 / 32767.0);
  fb.velocity = double(t_vel);

  if (buf[1] == static_cast<uint8_t>(Command::GET_FEEDBACK)) {
    uint8_t raw_pos_8 = buf[7];
    fb.position = double(raw_pos_8) * (360.0 / 255.0);
  } else {
    uint16_t t_pos = (buf[6] << 8) | buf[7];
    fb.position = double(t_pos) * (360.0 / 32767.0);
  }
  fb.error = buf[8];
  return fb;
}

void Communicator::switchMode(uint8_t id, Mode m) {
  uint8_t pkt[PACKET_SIZE] = {};
  pkt[0] = id;
  pkt[1] = static_cast<uint8_t>(Command::SWITCH_MODE);
  pkt[8] = static_cast<uint8_t>(m);
  pkt[9] = computeCRC(pkt, 9);
  std::cout << "SwitchMode packet: ";
  for (uint8_t byte : pkt) std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                                 << int(byte) << " ";
  std::cout << std::dec << std::endl;
  sendPacket(pkt);
}

Feedback Communicator::driveMotor(uint8_t id, int16_t val) {
  uint8_t pkt[PACKET_SIZE] = {};
  pkt[0] = id;
  pkt[1] = static_cast<uint8_t>(Command::DRIVE_MOTOR);
  pkt[2] = uint8_t((val >> 8) & 0xFF);
  pkt[3] = uint8_t(val & 0xFF);
  pkt[9] = computeCRC(pkt, 9);
  std::cout << "Driving packet: ";
  for (uint8_t byte : pkt) std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                                    << int(byte) << " ";
  std::cout << std::dec << std::endl;
  sendPacket(pkt);
  uint8_t buf[PACKET_SIZE] = {};
  readPacket(buf);
  return parseFeedback(buf);
}

Feedback Communicator::getFeedback(uint8_t id) {
  uint8_t pkt[PACKET_SIZE] = {};
  pkt[0] = id;
  pkt[1] = static_cast<uint8_t>(Command::GET_FEEDBACK);
  pkt[9] = computeCRC(pkt, 9);
  sendPacket(pkt);
  uint8_t buf[PACKET_SIZE] = {};
  readPacket(buf);
  return parseFeedback(buf);
}

void Communicator::setID(uint8_t /*old_id*/, uint8_t new_id) {
  uint8_t pkt[PACKET_SIZE] = {0xAA, 0x55, 0x53, new_id};
  pkt[9] = computeCRC(pkt, 9);
  for (int i = 0; i < 5; ++i) sendPacket(pkt);
}

uint8_t Communicator::queryID(uint8_t id) {
  uint8_t pkt[PACKET_SIZE] = {};
  pkt[0] = id;
  pkt[1] = static_cast<uint8_t>(Command::QUERY_ID);
  pkt[9] = computeCRC(pkt, 9);
  sendPacket(pkt);
  uint8_t resp[PACKET_SIZE] = {};
  readPacket(resp);
  return resp[3];
}

} // namespace ddsm115
