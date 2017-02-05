/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     DXL Monitor Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 1.0 and 2.0
// This example is tested with a Dynamixel MX-28, a Dynamixel PRO 54-200 and an USB2DYNAMIXEL
// Be sure that properties of Dynamixel MX and PRO are already set as %% MX - ID : 1 / Baudnum : 1 (Baudrate : 1000000) , PRO - ID : 1 / Baudnum : 3 (Baudrate : 1000000)
//

#if defined(__linux__) || defined(defined(__APPLE__))
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dynamixel_sdk.h"                                   // Uses Dynamixel SDK library

// Protocol version
#define PROTOCOL_VERSION1               1.0                 // See which protocol version is used in the Dynamixel
#define PROTOCOL_VERSION2               2.0

// Default setting
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

int getch()
{
#if defined(__linux__) || defined(defined(__APPLE__))
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(defined(__APPLE__))
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

void usage(char *progname)
{
  printf("-----------------------------------------------------------------------\n");
  printf("usage: %s\n" \
         " [-h | --help]........: display this help\n" \
         " [-d | --device]......: port to open                                   \n" \
         , progname);
  printf("-----------------------------------------------------------------------\n");
}

void help()
{
  printf("\n");
  printf("                    .----------------------------.\n");
  printf("                    |  DXL Monitor Command List  |\n");
  printf("                    '----------------------------'\n");
  printf(" =========================== Common Commands ===========================\n");
  printf(" \n");
  printf(" help|h|?                    :Displays help information\n");
  printf(" baud [BAUD_RATE]            :Changes baudrate to [BAUD_RATE] \n");
  printf("                               ex) baud 2400 (2400 bps) \n");
  printf("                               ex) baud 1000000 (1 Mbps)  \n");
  printf(" exit                        :Exit this program\n");
  printf(" scan                        :Outputs the current status of all Dynamixels\n");
  printf(" ping [ID] [ID] ...          :Outputs the current status of [ID]s \n");
  printf(" bp                          :Broadcast ping (Dynamixel Protocol 2.0 only)\n");
  printf(" \n");
  printf(" ==================== Commands for Dynamixel Protocol 1.0 ====================\n");
  printf(" \n");
  printf(" wrb1|w1 [ID] [ADDR] [VALUE] :write byte [VALUE] to [ADDR] of [ID]\n");
  printf(" wrw1 [ID] [ADDR] [VALUE]    :write word [VALUE] to [ADDR] of [ID]\n");
  printf(" rdb1 [ID] [ADDR]            :read byte value from [ADDR] of [ID]\n");
  printf(" rdw1 [ID] [ADDR]            :read word value from [ADDR] of [ID]\n");
  printf(" reset1|rst1 [ID]            :Factory reset the Dynamixel of [ID]\n");
  printf(" \n");
  printf(" ==================== Commands for Dynamixel Protocol 2.0 ====================\n");
  printf(" \n");
  printf(" wrb2|w2 [ID] [ADDR] [VALUE] :write byte [VALUE] to [ADDR] of [ID]\n");
  printf(" wrw2 [ID] [ADDR] [VALUE]    :write word [VALUE] to [ADDR] of [ID]\n");
  printf(" wrd2 [ID] [ADDR] [VALUE]    :write dword [VALUE] to [ADDR] of [ID]\n");
  printf(" rdb2 [ID] [ADDR]            :read byte value from [ADDR] of [ID]\n");
  printf(" rdw2 [ID] [ADDR]            :read word value from [ADDR] of [ID]\n");
  printf(" rdd2 [ID] [ADDR]            :read dword value from [ADDR] of [ID]\n");
  printf(" reboot2|rbt2 [ID]           :reboot the Dynamixel of [ID]\n");
  printf(" reset2|rst2 [ID] [OPTION]   :Factory reset the Dynamixel of [ID]\n");
  printf("                               OPTION: 255(All), 1(Except ID), 2(Except ID&Baud)\n");

  printf("\n");
}

void scan(int port_num, int protocol_version1, int protocol_version2)
{
	int id;
  uint16_t dxl_model_num;
  int dxl_comm_result;

  fprintf(stderr, "\n");
  fprintf(stderr, "scan Dynamixel Using Protocol 1.0\n");
  for (id = 1; id < 253; id++)
  {
    dxl_model_num = pingGetModelNum(port_num, protocol_version1, id);
    if ((dxl_comm_result = getLastTxRxResult(port_num, protocol_version1)) == COMM_SUCCESS)
    {
      fprintf(stderr, "\n                                          ... SUCCESS \r");
      fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", id, dxl_model_num);
    }
    else
    {
      fprintf(stderr, ".");
    }

    if (kbhit())
    {
      char c = getch();
      if (c == 0x1b)
        break;
    }
  }
  fprintf(stderr, "\n\n");

  fprintf(stderr, "scan Dynamixel Using Protocol 2.0\n");
  for (id = 1; id < 253; id++)
  {
  	dxl_model_num = pingGetModelNum(port_num, protocol_version2, id);
    if ((dxl_comm_result = getLastTxRxResult(port_num, protocol_version2)) == COMM_SUCCESS)
  	{
      fprintf(stderr, "\n                                          ... SUCCESS \r");
      fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", id, dxl_model_num);
    }
    else
    {
      fprintf(stderr, ".");
    }

    if (kbhit())
    {
      char c = getch();
      if (c == 0x1b)
        break;
    }
  }
  fprintf(stderr, "\n\n");
}

void writeNByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t addr, uint16_t length, uint32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    write1ByteTxRx(port_num, protocol_version, id, addr, (uint8_t)value);
  }
  else if (length == 2)
  {
    write2ByteTxRx(port_num, protocol_version, id, addr, (uint16_t)value);
  }
  else if (length == 4)
  {
    write4ByteTxRx(port_num, protocol_version, id, addr, (uint32_t)value);
  }
  if ((dxl_comm_result = getLastTxRxResult(port_num, protocol_version)) == COMM_SUCCESS)
  {
    if ((dxl_error = getLastRxPacketError(port_num, protocol_version)) != 0)
			printRxPacketError(protocol_version, dxl_error);
    fprintf(stderr, "\n Success to write\n\n");
  }
  else
  {
    printTxRxResult(protocol_version, dxl_comm_result);
    fprintf(stderr, "\n Fail to write! \n\n");
  }
}

void readNByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t addr, uint16_t length)
{
  uint8_t dxl_error = 0;
  int     dxl_comm_result = COMM_TX_FAIL;

  int8_t  value8    = 0;
  int16_t value16   = 0;
  int32_t value32   = 0;

  if (length == 1)
  {
    value8 = read1ByteTxRx(port_num, protocol_version, id, addr);
  }
  else if (length == 2)
  {
    value16 = read2ByteTxRx(port_num, protocol_version, id, addr);
  }
  else if (length == 4)
  {
    value32 = read4ByteTxRx(port_num, protocol_version, id, addr);
  }

  if ((dxl_comm_result = getLastTxRxResult(port_num, protocol_version)) == COMM_SUCCESS)
  {
    if ((dxl_error = getLastRxPacketError(port_num, protocol_version)) != 0)
			printRxPacketError(protocol_version, dxl_error);

    if (length == 1)
    {
      fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint8_t)value8, value8);
    }
    else if (length == 2)
    {
      fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint16_t)value16, value16);
    }
    else if (length == 4)
    {
      fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint32_t)value32, value32);
    }
  }
  else
  {
    printTxRxResult(protocol_version, dxl_comm_result);
    fprintf(stderr, "\n Fail to write! \n\n");
  }
}

int main(int argc, char *argv[])
{
  fprintf(stderr, "\n***********************************************************************\n");
  fprintf(stderr,   "*                            DXL Monitor                              *\n");
  fprintf(stderr,   "***********************************************************************\n\n");

  char *dev_name = (char*)DEVICENAME;

#if defined(__linux__) || defined(defined(__APPLE__))
  /* parameter parsing */
  while(1)
  {
    int option_index = 0, c = 0;
    static struct option long_options[] = {
        {"h", no_argument, 0, 0},
        {"help", no_argument, 0, 0},
        {"d", required_argument, 0, 0},
        {"device", required_argument, 0, 0},
        {0, 0, 0, 0}
    };

    /* parsing all parameters according to the list above is sufficent */
    c = getopt_long_only(argc, argv, "", long_options, &option_index);

    /* no more options to parse */
    if (c == -1) break;

    /* unrecognized option */
    if (c == '?')
    {
      usage(argv[0]);
      return 0;
    }

    /* dispatch the given options */
    switch (option_index)
    {
      /* h, help */
      case 0:
      case 1:
        usage(argv[0]);
        return 0;
        break;

      /* d, device */
      case 2:
      case 3:
        if (strlen(optarg) == 1)
        {
          char tmp[20];
          sprintf(tmp, "/dev/ttyUSB%s", optarg);
          dev_name = strdup(tmp);
        }
        else
        {
          dev_name = strdup(optarg);
        }
        break;

        default:
          usage(argv[0]);
          return 0;
    }
  }
#endif

  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(dev_name);

  // Initialize PacketHandler Structs
  packetHandler();

  // Open port
  if (openPort(port_num))
  {
    printf("Succeeded to open the port!\n\n");
    printf(" - Device Name : %s\n", dev_name);
    printf(" - Baudrate    : %d\n\n", getBaudRate(port_num));
  }
  else
  {
    printf("Failed to open the port! [%s]\n", dev_name);
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  char    input[128];
  char    cmd[80];
  char    param[20][30];
  int     num_param;
  char*   token;
  uint8_t dxl_error;

  while(1)
  {
    printf("[CMD] ");
    fgets(input, sizeof(input), stdin);
    char *p;
    if ((p = strchr(input, '\n'))!= NULL) *p = '\0';
    fflush(stdin);

    if (strlen(input) == 0)
      continue;

    token = strtok(input, " ");
    if (token == 0)
      continue;

    strcpy(cmd, token);
    token = strtok(0, " ");
    num_param = 0;
    while(token != 0)
    {
      strcpy(param[num_param++], token);
      token = strtok(0, " ");
    }

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0)
    {
      help();
    }
    else if (strcmp(cmd, "baud") == 0)
    {
      if (num_param == 1)
      {
        if (setBaudRate(port_num, atoi(param[0])) == False)
        {
          fprintf(stderr, " Failed to change baudrate! \n");
        }
        else
        {
          fprintf(stderr, " Success to change baudrate! [ BAUD RATE: %d ]\n", atoi(param[0]));
        }
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
        continue;
      }
    }
    else if (strcmp(cmd, "exit") == 0)
    {
      closePort(port_num);
      return 0;
    }
    else if (strcmp(cmd, "scan") == 0)
    {
      scan(port_num, PROTOCOL_VERSION1, PROTOCOL_VERSION2);
    }
    else if (strcmp(cmd, "ping") == 0)
    {
      int i;
      uint16_t dxl_model_num;
      int dxl_comm_result;

      if (num_param == 0)
      {
        fprintf(stderr, " Invalid parameters! \n");
        continue;
      }

      fprintf(stderr, "\n");
      fprintf(stderr, "ping Using Protocol 1.0\n");
      for (i = 0; i < num_param; i++)
      {
        dxl_model_num = pingGetModelNum(port_num, PROTOCOL_VERSION1, atoi(param[i]));
				if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION1)) == COMM_SUCCESS)
        {
          fprintf(stderr, "\n                                          ... SUCCESS \r");
          fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", atoi(param[i]), dxl_model_num);
        }
        else
        {
          fprintf(stderr, "\n                                          ... FAIL \r");
          fprintf(stderr, " [ID:%.3d] \n", atoi(param[i]));
        }
      }
      fprintf(stderr, "\n");

      fprintf(stderr, "\n");
      fprintf(stderr, "ping Using Protocol 2.0\n");
      for (i = 0; i < num_param; i++)
      {
      	dxl_model_num = pingGetModelNum(port_num, PROTOCOL_VERSION2, atoi(param[i]));
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION2)) == COMM_SUCCESS)
        {
          fprintf(stderr, "\n                                          ... SUCCESS \r");
          fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", atoi(param[i]), dxl_model_num);
        }
        else
        {
          fprintf(stderr, "\n                                          ... FAIL \r");
          fprintf(stderr, " [ID:%.3d] \n", atoi(param[i]));
        }
      }
      fprintf(stderr, "\n");
    }
    else if (strcmp(cmd, "bp") == 0)
    {
      if (num_param == 0)
      {
        int dxl_comm_result, id;

        broadcastPing(port_num, PROTOCOL_VERSION2);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION2)) != COMM_SUCCESS)
          printTxRxResult(PROTOCOL_VERSION2, dxl_comm_result);

        printf("Detected Dynamixel : \n");
        for (id = 0; id < MAX_ID; id++)
        {
          if (getBroadcastPingResult(port_num, id, PROTOCOL_VERSION2))
            printf("[ID:%03d]\n", id);
        }
        printf("\n");
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }

    }
    else if (strcmp(cmd, "wrb1") == 0 || strcmp(cmd, "w1") == 0)
    {
      if (num_param == 3)
      {
        writeNByteTxRx(port_num, PROTOCOL_VERSION1, atoi(param[0]), atoi(param[1]), 1, atoi(param[2]));
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "wrb2") == 0 || strcmp(cmd, "w2") == 0)
    {
      if (num_param == 3)
      {
        writeNByteTxRx(port_num, PROTOCOL_VERSION2, atoi(param[0]), atoi(param[1]), 1, atoi(param[2]));
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "wrw1") == 0)
    {
      if (num_param == 3)
      {
        writeNByteTxRx(port_num, PROTOCOL_VERSION1, atoi(param[0]), atoi(param[1]), 2, atoi(param[2]));
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "wrw2") == 0)
    {
      if (num_param == 3)
      {
        writeNByteTxRx(port_num, PROTOCOL_VERSION2, atoi(param[0]), atoi(param[1]), 2, atoi(param[2]));
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "wrd2") == 0)
    {
      if (num_param == 3)
      {
        writeNByteTxRx(port_num, PROTOCOL_VERSION2, atoi(param[0]), atoi(param[1]), 4, atoi(param[2]));
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "rdb1") == 0)
    {
      if (num_param == 2)
      {
        readNByteTxRx(port_num, PROTOCOL_VERSION1, atoi(param[0]), atoi(param[1]), 1);
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "rdb2") == 0)
    {
      if (num_param == 2)
      {
        readNByteTxRx(port_num, PROTOCOL_VERSION2, atoi(param[0]), atoi(param[1]), 1);
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "rdw1") == 0)
    {
      if (num_param == 2)
      {
        readNByteTxRx(port_num, PROTOCOL_VERSION1, atoi(param[0]), atoi(param[1]), 2);
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "rdw2") == 0)
    {
      if (num_param == 2)
      {
        readNByteTxRx(port_num, PROTOCOL_VERSION2, atoi(param[0]), atoi(param[1]), 2);
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "rdd2") == 0)
    {
      if (num_param == 2)
      {
        readNByteTxRx(port_num, PROTOCOL_VERSION2, atoi(param[0]), atoi(param[1]), 4);
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "reboot2") == 0 || strcmp(cmd, "rbt2") == 0)
    {
      if (num_param == 1)
      {
        int dxl_comm_result;
        reboot(port_num, PROTOCOL_VERSION2, atoi(param[0]));
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION2)) == COMM_SUCCESS)
        {
          if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION2)) != 0)
            printRxPacketError(PROTOCOL_VERSION2, dxl_error);
          fprintf(stderr, "\n Success to reboot! \n\n");
        }
        else
        {
          printTxRxResult(PROTOCOL_VERSION2, dxl_comm_result);
          fprintf(stderr, "\n Fail to reboot! \n\n");
        }
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "reset1") == 0 || strcmp(cmd, "rst1") == 0)
    {
      if (num_param == 1)
      {
        int dxl_comm_result;
        factoryReset(port_num, PROTOCOL_VERSION1, atoi(param[0]), 0x00);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION1)) == COMM_SUCCESS)
        {
          if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION1)) != 0)
            printRxPacketError(PROTOCOL_VERSION1, dxl_error);
          fprintf(stderr, "\n Success to reset! \n\n");
        }
        else
        {
          printTxRxResult(PROTOCOL_VERSION1, dxl_comm_result);
          fprintf(stderr, "\n Fail to reset! \n\n");
        }
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "reset2") == 0 || strcmp(cmd, "rst2") == 0)
    {
      if (num_param == 2)
      {
        int dxl_comm_result;
        factoryReset(port_num, PROTOCOL_VERSION2, atoi(param[0]), 0x00);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION2)) == COMM_SUCCESS)
        {
          if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION2)) != 0)
            printRxPacketError(PROTOCOL_VERSION2, dxl_error);
          fprintf(stderr, "\n Success to reset! \n\n");
        }
        else
        {
          printTxRxResult(PROTOCOL_VERSION2, dxl_comm_result);
          fprintf(stderr, "\n Fail to reset! \n\n");
        }
      }
      else
      {
        fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else
    {
      printf(" Bad command! Please input 'help'.\n");
    }
  }
}
