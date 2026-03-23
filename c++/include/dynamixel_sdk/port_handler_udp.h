/*******************************************************************************
 * EthernetToDynamixel - PortHandlerUDP
 *
 * UDP-based PortHandler for DynamixelSDK.
 * Communicates with a Nucleo-F429ZI UDP-to-UART bridge.
 *
 * Port name format:  "udp:<ip>:<port>"
 * Example:           "udp:169.254.1.2:8001"
 *
 * Data ports (8001-8006): raw Dynamixel bytes, zero overhead.
 * Control port (9001):    baud rate changes, status queries.
 ******************************************************************************/

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORT_HANDLER_UDP_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORT_HANDLER_UDP_H_

#include "port_handler.h"
#include <string>

#define BRIDGE_BASE_PORT  8001
#define BRIDGE_CTRL_PORT  9001

namespace dynamixel
{

class WINDECLSPEC PortHandlerUDP : public PortHandler
{
public:
  PortHandlerUDP(const char *port_name);
  virtual ~PortHandlerUDP();

  bool    openPort()                          override;
  void    closePort()                         override;
  void    clearPort()                         override;

  void    setPortName(const char *port_name)  override;
  char   *getPortName()                       override;

  bool    setBaudRate(const int baudrate)      override;
  int     getBaudRate()                        override;

  int     getBytesAvailable()                  override;
  int     readPort(uint8_t *packet, int length) override;
  int     writePort(uint8_t *packet, int length) override;

  void    setPacketTimeout(uint16_t packet_length) override;
  void    setPacketTimeout(double msec)            override;
  bool    isPacketTimeout()                        override;

private:
  int         socket_fd_;
  int         ctrl_socket_fd_;
  int         baudrate_;
  char        port_name_[200];
  std::string target_ip_;
  uint16_t    target_port_;
  uint8_t     channel_index_;

  double      packet_start_time_;
  double      packet_timeout_;
  double      tx_time_per_byte_;

  uint8_t     recv_buf_[2048];
  int         recv_buf_len_;
  int         recv_buf_pos_;

  bool        parsePortName(const char *port_name);
  double      getCurrentTime();
  double      getTimeSinceStart();
};

}

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORT_HANDLER_UDP_H_ */
