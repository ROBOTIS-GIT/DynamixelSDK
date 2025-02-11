^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_sdk
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.7.51 (2021-04-14)
-------------------
* Add Sync / Bulk read write ROS examples
* Contributors: JaehyunShim

3.7.41 (2020-08-12)
-------------------
* DYNAMIXEL SDK ROS example initial commit.
* Bug fix for 4.5Mbps support (#430)
* Contributors: Zerom, Will Son

3.7.31 (2020-07-13)
-------------------
* ROS 1 Noetic Ninjemys support
* 3x faster getError member function of GroupSyncRead Class
* Contributors: developer0hye, Zerom, Will Son

3.7.21 (2019-09-06)
-------------------
* Fixed buffer overflow bug (rxpacket size)
* Fixed typo in the package.xml and header files
* Contributors: Chris Lalancette, Zerom, Pyo

3.7.11 (2019-08-19)
-------------------
- Updated C lib and DLL file
- Changed C# / win32 / protocol_combined output path
- Fixed "protocol_combined" example bug
- Fixed typo in bulk_read_write.py
* Contributors: Pyo, Zerom

3.7.10 (2019-08-19)
-------------------
* Supported ROS 2 Dashing Diademata
* Contributors: Darby, Pyo

3.7.0 (2019-01-03)
------------------
* Added clear instruction `#269 <https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/269>`_
* Removed busy waiting for rxPacket()
* Fixed addStuffing() function (reduced stack memory usage)
* Fixed memory issues `#268 <https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/268>`_
* Fixed the broadcast ping bug in dxl_monitor
* Contributors: Gilbert, Pyo, Zerom

3.6.2 (2018-07-17)
------------------
* Added python modules for ROS to ros folder
* Moved cpp library files for ROS to ros folder
* Created an ROS package separately `#187 <https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/187>`_
* Modified the e-Manual address to emanual.robotis.com
* Contributors: Pyo

3.6.1 (2018-06-14)
------------------
* removed printTxRxResult(), printRxPacketError() `#193 <https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/193>`_
* removed cache files
* merge pull request `#195 <https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/195>`_
* Contributors: Gilbert, Pyo

3.6.0 (2018-03-16)
------------------
* Replaced : DynamixelSDK Python as a native language (Python 2 and 3 for Windows, Linux, Mac OS X) #93 #122 #147 #181 #182 #185
* Added : CONTRIBUTING.md added
* Changes : ISSUE_TEMPLATE.md modified
* Changes : C++ version - SyncRead / BulkRead - getError functions added
* Changes : Deprecated functions removed
* Fixes : DynamixelSDK MATLAB 2017 - new typedef (int8_t / int16_t / int32_t) applied in robotis_def.h #161 #179
* Fixes : Added missing header file for reset and factory_reset examples #167
* Contributors: Leon

3.5.4 (2017-12-01)
------------------
* Added : Deprecated is now being shown by attributes #67 #107
* Fixes : DynamixelSDK ROS Indigo Issue - target_sources func in CMake
* Fixes : Bug in protocol1_packet_handler.cpp, line 222 checking the returned Error Mask #120
* Fixes : Packet Handlers - array param uint8_t to uint16_t to avoid closure loop when the packet is too long to be in uint8_t appropriately
* Fixes : Group Syncwrite using multiple ports in c library issue solved (test code is also in this issue bulletin) #124
* Fixes : Support getting of time on MacOSX/XCode versions that doesn't support (CLOCK_REALTIME issue) #141 #144
* Changes : DynamixelSDK Ubuntu Linux usb ftdi latency timer fix issue - changes the default latency timer as 16 ms in all OS, but some about how to change the latency timer was commented in the codes (now the latency timer should be adjusted by yourself... see port_handler_linux source code to see details) #116
* Contributors: Leon

3.5.3 (2017-10-30)
------------------
* Fixes : DynamixelSDK ROS Kinetic Issue - ARM - Debian Jessie solved by replacing target_sources func in CMake to set_property #136
* Contributors: Leon

3.5.2 (2017-09-18)
------------------
* Recover : Check if the id of rxpacket is the same as the id of txpacket #82
* Changes : Ping examples now will not show Dynamixel model number when communication is failed
* Contributors: Leon

3.5.1 (2017-08-18)
------------------
* Standardizes folder structure of ROS c++
* Fixes : Inconvenient way of getting meaning of packet result and error value #67
* Fixes : Maximum length of port name is expanded to 100 #100
* Alternative : Include port_handler.h through relative path. #90
* Changes : Indent correction / Example tests & refresh / OS IFDEF
* Changes : Default Baudrate from 1000000(1M) bps to 57600 bps
* Changes : Macro for control table value changed to uints
* Changes : API references will be provided as doxygen (updates in c++ @ 3.5.1)
* Changes : License changed into Apache License .2.0 (Who are using SDK in previous license can use it as it is)
* Deprecated : getTxRxResult, getRxPacketError function will be unavailable in Dynamixel SDK 3.6.1
* Contributors: Leon

3.4.7 (2017-07-18)
------------------
* hotfix - Bug in Dynamixel group control is solved temporarily
* Contributors: Leon, Zerom

3.4.6 (2017-07-07)
------------------
* hotfix - now DynamixelSDK for protocol1.0 supports read/write 4Byte (for XM series)
* Contributors: Leon

3.4.5 (2017-05-23)
------------------
* added option to check if the id of rxpacket is the same as the id of txpacket.
* Contributors: Leon, Zerom

3.4.4 (2017-04-26)
------------------
* hotfix - return delay time is changed from 4 into 8 due to the Ubuntu update 16.04.2
* Contributors: Leon

3.4.3 (2017-02-17)
------------------
* DynamixelSDK C++ ver. and ROS ver. in Windows platform now can use the port number of over then 10 #45
* Contributors: Leon

3.4.2 (2017-02-16)
------------------
* fprintf output in GrouBulkRead of C++ removed
* MATLAB library compiler error solving
* Makefile for build example sources in SBC added
* build files of windows c and c++ SDK rebuilt by using renewed SDK libraries
* example source of dxl_monitor - c and cpp ver modified #50
* Solved issue : #31, #34, #36, #50
* Contributors: Leon

3.4.1 (2016-08-22)
------------------
* added ROS package folder for ROS users
* modified original header files for ROS package
* Contributors: Leon

3.4.0 (2016-08-12)
------------------
* first public release for Kinetic
* added package information for wrapping version for ROS
* added ROS catkin package files.
* linux build file for SBC
* License marks for example codes
* Resource Files comments Korean -> English
* Update Makefile
* Update Makefile
* comments modified & aligned
* Release folders in c++ example removed & dxl_monitor.cpp Capital function name modified as ROS c++ code style & included file paths of packet/port handler in dynamixel_sdk.h removed and added parent header file
* Update dxl_monitor.cpp
* file opened
* folder name modification error solved
* License specified
* Code Style modified into ROS C++ coding style
  Function & File Names changed into underscored
* Group Bulk/Sync class ClearParam() function changed.
* dll file name changed
* dll file name changed
* Comment modified
* [Protocol1PacketHandler]
  RxPacket packet length re-calculate bug fixed.
* [Protocol2PacketHandler]
  RxPacket packet length re-calculate bug fixed.
* Makefile updated
  Source reorganization
* Windows version updated
  Makefile modified
  Source reorganization
* GroupBulkRead : GetData function bug fixed.
* [GroupBulkRead / GroupSyncRead]
  added IsAvailable() function
  modified GetData() function
* GetData() function changed.
* reducing the count of calling MakeParam function
* added rxpacket error check
* ReadTxRx function modified. (to use TxRxPacket function)
* DXL Monitor program arguments added.
* if the last bulk_read / sync_read result is failure -> GetData return false
* communication result & rx packet error print function modified.
* first release
* Contributors: Leon, Zerom, Pyo
