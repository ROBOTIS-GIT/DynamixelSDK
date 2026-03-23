/*******************************************************************************
 * EthernetToDynamixel - PortHandlerUDP
 *
 * UDP-based PortHandler for DynamixelSDK.
 * Communicates with a Nucleo-F429ZI UDP-to-UART bridge.
 *
 * Data ports (8001-8006): raw Dynamixel bytes, zero overhead.
 * Control port (9001):    [channel][cmd][payload...]
 ******************************************************************************/

#if defined(__linux__) || defined(__APPLE__)

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "port_handler_udp.h"

#define CTRL_CMD_BAUD_RATE  0x01
#define CTRL_CMD_STATUS     0x02
#define UDP_TIMEOUT_MS      100

using namespace dynamixel;

PortHandlerUDP::PortHandlerUDP(const char *port_name)
  : socket_fd_(-1),
    ctrl_socket_fd_(-1),
    baudrate_(1000000),
    target_port_(0),
    channel_index_(0),
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte_(0.0),
    recv_buf_len_(0),
    recv_buf_pos_(0)
{
  is_using_ = false;
  setPortName(port_name);
}

PortHandlerUDP::~PortHandlerUDP()
{
  closePort();
}

/* ---------- Port name parsing -------------------------------------------- */

bool PortHandlerUDP::parsePortName(const char *port_name)
{
  if (strncmp(port_name, "udp:", 4) != 0) {
    printf("[PortHandlerUDP] Invalid port name format. Expected udp:<ip>:<port>\n");
    return false;
  }

  const char *rest = port_name + 4;
  const char *colon = strrchr(rest, ':');
  if (!colon || colon == rest) {
    printf("[PortHandlerUDP] Cannot parse IP:port from '%s'\n", port_name);
    return false;
  }

  target_ip_.assign(rest, colon - rest);
  target_port_ = (uint16_t)atoi(colon + 1);

  if (target_port_ == 0) {
    printf("[PortHandlerUDP] Invalid port number\n");
    return false;
  }

  channel_index_ = (uint8_t)(target_port_ - BRIDGE_BASE_PORT);

  return true;
}

/* ---------- Open / Close ------------------------------------------------- */

static int create_connected_udp(const char *ip, uint16_t port)
{
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return -1;

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(port);
  if (inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
    close(fd);
    return -1;
  }

  if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(fd);
    return -1;
  }

  return fd;
}

bool PortHandlerUDP::openPort()
{
  if (!parsePortName(port_name_))
    return false;

  closePort();

  socket_fd_ = create_connected_udp(target_ip_.c_str(), target_port_);
  if (socket_fd_ < 0) {
    printf("[PortHandlerUDP] Failed to create data socket: %s\n", strerror(errno));
    return false;
  }

  ctrl_socket_fd_ = create_connected_udp(target_ip_.c_str(), BRIDGE_CTRL_PORT);
  if (ctrl_socket_fd_ < 0) {
    printf("[PortHandlerUDP] Failed to create control socket: %s\n", strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  struct timeval tv;
  tv.tv_sec  = 0;
  tv.tv_usec = UDP_TIMEOUT_MS * 1000;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  int flags = fcntl(socket_fd_, F_GETFL, 0);
  fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

  tx_time_per_byte_ = (1000.0 / (double)baudrate_) * 10.0;

  printf("[PortHandlerUDP] Opened %s:%d (channel %d, ctrl %d)\n",
         target_ip_.c_str(), target_port_, channel_index_, BRIDGE_CTRL_PORT);
  return true;
}

void PortHandlerUDP::closePort()
{
  if (socket_fd_ != -1) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  if (ctrl_socket_fd_ != -1) {
    close(ctrl_socket_fd_);
    ctrl_socket_fd_ = -1;
  }
}

void PortHandlerUDP::clearPort()
{
  if (socket_fd_ < 0) return;

  uint8_t trash[2048];
  while (recv(socket_fd_, trash, sizeof(trash), MSG_DONTWAIT) > 0)
    ;

  recv_buf_len_ = 0;
  recv_buf_pos_ = 0;
}

/* ---------- Port name / baud --------------------------------------------- */

void PortHandlerUDP::setPortName(const char *port_name)
{
  strncpy(port_name_, port_name, sizeof(port_name_) - 1);
  port_name_[sizeof(port_name_) - 1] = '\0';
}

char *PortHandlerUDP::getPortName()
{
  return port_name_;
}

bool PortHandlerUDP::setBaudRate(const int baudrate)
{
  if (ctrl_socket_fd_ < 0)
    return false;

  uint8_t cmd[6];
  cmd[0] = channel_index_;
  cmd[1] = CTRL_CMD_BAUD_RATE;
  uint32_t baud_le = (uint32_t)baudrate;
  memcpy(cmd + 2, &baud_le, 4);

  struct timeval tv;
  tv.tv_sec  = 1;
  tv.tv_usec = 0;
  setsockopt(ctrl_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  ssize_t sent = send(ctrl_socket_fd_, cmd, 6, 0);
  if (sent != 6) {
    printf("[PortHandlerUDP] Failed to send baud rate command\n");
    return false;
  }

  uint8_t ack[3];
  ssize_t n = recv(ctrl_socket_fd_, ack, sizeof(ack), 0);

  if (n == 3 && ack[0] == channel_index_ && ack[1] == CTRL_CMD_BAUD_RATE && ack[2] == 0x00) {
    baudrate_ = baudrate;
    tx_time_per_byte_ = (1000.0 / (double)baudrate_) * 10.0;
    printf("[PortHandlerUDP] Baud rate set to %d\n", baudrate);
    return true;
  }

  printf("[PortHandlerUDP] Baud rate change failed (n=%zd)\n", n);
  return false;
}

int PortHandlerUDP::getBaudRate()
{
  return baudrate_;
}

/* ---------- Read / Write (raw, no prefix) -------------------------------- */

int PortHandlerUDP::getBytesAvailable()
{
  if (recv_buf_pos_ < recv_buf_len_)
    return recv_buf_len_ - recv_buf_pos_;

  ssize_t n = recv(socket_fd_, recv_buf_, sizeof(recv_buf_), MSG_DONTWAIT);
  if (n > 0) {
    recv_buf_len_ = (int)n;
    recv_buf_pos_ = 0;
    return (int)n;
  }

  return 0;
}

int PortHandlerUDP::readPort(uint8_t *packet, int length)
{
  if (recv_buf_pos_ < recv_buf_len_) {
    int available = recv_buf_len_ - recv_buf_pos_;
    int to_copy = (length < available) ? length : available;
    memcpy(packet, recv_buf_ + recv_buf_pos_, to_copy);
    recv_buf_pos_ += to_copy;
    return to_copy;
  }

  double remaining = packet_timeout_ - getTimeSinceStart();
  if (remaining <= 0)
    remaining = 1.0;

  struct timeval tv;
  tv.tv_sec  = (long)(remaining / 1000.0);
  tv.tv_usec = (long)((remaining - tv.tv_sec * 1000.0) * 1000.0);
  if (tv.tv_sec == 0 && tv.tv_usec < 1000)
    tv.tv_usec = 1000;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  int flags = fcntl(socket_fd_, F_GETFL, 0);
  fcntl(socket_fd_, F_SETFL, flags & ~O_NONBLOCK);

  ssize_t n = recv(socket_fd_, recv_buf_, sizeof(recv_buf_), 0);

  fcntl(socket_fd_, F_SETFL, flags);

  if (n > 0) {
    recv_buf_len_ = (int)n;
    recv_buf_pos_ = 0;

    int to_copy = (length < (int)n) ? length : (int)n;
    memcpy(packet, recv_buf_ + recv_buf_pos_, to_copy);
    recv_buf_pos_ += to_copy;
    return to_copy;
  }

  return 0;
}

int PortHandlerUDP::writePort(uint8_t *packet, int length)
{
  if (socket_fd_ < 0 || length <= 0)
    return -1;

  ssize_t sent = send(socket_fd_, packet, length, 0);
  if (sent < 0)
    return -1;

  recv_buf_len_ = 0;
  recv_buf_pos_ = 0;

  return (int)sent;
}

/* ---------- Timeout ------------------------------------------------------ */

void PortHandlerUDP::setPacketTimeout(uint16_t packet_length)
{
  packet_start_time_ = getCurrentTime();
  packet_timeout_    = (tx_time_per_byte_ * (double)packet_length) + 5.0;
}

void PortHandlerUDP::setPacketTimeout(double msec)
{
  packet_start_time_ = getCurrentTime();
  packet_timeout_    = msec;
}

bool PortHandlerUDP::isPacketTimeout()
{
  if (getTimeSinceStart() > packet_timeout_) {
    packet_timeout_ = 0;
    return true;
  }
  return false;
}

double PortHandlerUDP::getCurrentTime()
{
  struct timespec tv;
  clock_gettime(CLOCK_REALTIME, &tv);
  return ((double)tv.tv_sec * 1000.0 + (double)tv.tv_nsec * 0.001 * 0.001);
}

double PortHandlerUDP::getTimeSinceStart()
{
  double time = getCurrentTime() - packet_start_time_;
  if (time < 0.0)
    packet_start_time_ = getCurrentTime();
  return time;
}

#endif /* __linux__ || __APPLE__ */
