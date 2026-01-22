/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     DXL Monitor Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 1.0 and 2.0
// This example is tested with a Dynamixel MX-28, a Dynamixel PRO 54-200 and an USB2DYNAMIXEL
// Be sure that properties of Dynamixel MX and PRO are already set as %% MX - ID : 1 / Baudnum : 34 (Baudrate : 57600) , PRO - ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <getopt.h>
#include <termios.h>
#define STDIN_FILENO 0
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
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
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
#if defined(__linux__) || defined(__APPLE__)
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
  printf("Usage: %s\n", progname);
  printf(" [-h | --help]........: display this help\n");
  printf(" [-d | --device]......: port to open\n");
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
  printf("                               ex) baud 57600 (57600 bps) \n");
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
			printf("%s\n", getRxPacketError(protocol_version, dxl_error));
    fprintf(stderr, "\n Success to write\n\n");
  }
  else
  {
    printf("%s\n", getTxRxResult(protocol_version, dxl_comm_result));
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
			printf("%s\n", getRxPacketError(protocol_version, dxl_error));

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
    printf("%s\n", getTxRxResult(protocol_version, dxl_comm_result));
    fprintf(stderr, "\n Fail to write! \n\n");
  }
}

// [Add] Helper function to clear input buffer portably
void clean_stdin(void)
{
    int c;
    do {
        c = getchar();
    } while (c != '\n' && c != EOF);
}

int main(int argc, char *argv[])
{
  fprintf(stderr, "\n***********************************************************************\n");
  fprintf(stderr,   "*                            DXL Monitor                              *\n");
  fprintf(stderr,   "***********************************************************************\n\n");

  char *dev_name = (char*)DEVICENAME;

#if defined(__linux__) || defined(__APPLE__)
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
  int port_num = portHandler("/dev/ttyUSB0");
  
  // 디버깅용 출력 추가
  printf("DEBUG: port_num = %d\n", port_num);

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

    if (fgets(input, sizeof(input), stdin) == NULL) break;
    
    char *p = strchr(input, '\n');
    if (p != NULL) *p = '\0';
    else clean_stdin();

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
          printf("%s\n", getTxRxResult(PROTOCOL_VERSION2, dxl_comm_result));

        printf("Detected Dynamixel : \n");
        for (id = 0; id < MAX_ID; id++)
        {
          if (getBroadcastPingResult(port_num, PROTOCOL_VERSION2, id))
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
            printf("%s\n", getRxPacketError(PROTOCOL_VERSION2, dxl_error));
          fprintf(stderr, "\n Success to reboot! \n\n");
        }
        else
        {
          printf("%s\n", getTxRxResult(PROTOCOL_VERSION2, dxl_comm_result));
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
            printf("%s\n", getRxPacketError(PROTOCOL_VERSION1, dxl_error));
          fprintf(stderr, "\n Success to reset! \n\n");
        }
        else
        {
          printf("%s\n", getTxRxResult(PROTOCOL_VERSION1, dxl_comm_result));
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
            printf("%s\n", getRxPacketError(PROTOCOL_VERSION2, dxl_error));
          fprintf(stderr, "\n Success to reset! \n\n");
        }
        else
        {
          printf("%s\n", getTxRxResult(PROTOCOL_VERSION2, dxl_comm_result));
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
