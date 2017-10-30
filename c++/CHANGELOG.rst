^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_sdk
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.5.3 (2017-10-30)
-----------
* Fixes : DynamixelSDK ROS Kinetic Issue - ARM - Debian Jessie solved by replacing target_sources func in CMake to set_property #136
* Contributors: Leon

3.5.2 (2017-09-18)
-----------
* Recover : Check if the id of rxpacket is the same as the id of txpacket #82
* Changes : Ping examples now will not show Dynamixel model number when communication is failed
* Contributors: Leon

3.5.1 (2017-08-18)
-----------
* Standardizes folder structure of ROS c++
* Fixes : Inconvenient way of getting meaning of packet result and error value #67
* Fixes : Maximum length of port name is expanded to 100 #100
* Alternative : Include port_handler.h through relative path. #90
* Changes : Indent correction / Example tests & refresh / OS IFDEF
* Changes : Default Baudrate from 1000000(1M) bps to 57600 bps
* Changes : Macro for control table value changed to uints
* Changes : API references will be provided as doxygen (updates in c++ @ 3.5.1)
* Changes : License changed into Apache License .2.0 (Who are using SDK in previous license can use it as it is)
* Deprecated : printTxRxResult, printRxPacketError function will be unavailable in Dynamixel SDK 3.6.1
* Contributors: Leon

3.4.7 (2017-07-18)
-----------
* hotfix - Bug in Dynamixel group control is solved temporarily
* Contributors: Leon, Zerom

3.4.6 (2017-07-07)
-----------
* hotfix - now DynamixelSDK for protocol1.0 supports read/write 4Byte (for XM series)
* Contributors: Leon

3.4.5 (2017-05-23)
-----------
* added option to check if the id of rxpacket is the same as the id of txpacket.
* Contributors: Leon, Zerom

3.4.4 (2017-04-26)
-----------
* hotfix - return delay time is changed from 4 into 8 due to the Ubuntu update 16.04.2
* Contributors: Leon

3.4.3 (2017-02-17)
-----------
* DynamixelSDK C++ ver. and ROS ver. in Windows platform now can use the port number of over then 10 #45
* Contributors: Leon

3.4.2 (2017-02-16)
-----------
* fprintf output in GrouBulkRead of C++ removed
* MATLAB library compiler error solving
* Makefile for build example sources in SBC added
* build files of windows c and c++ SDK rebuilt by using renewed SDK libraries
* example source of dxl_monitor - c and cpp ver modified #50
* Solved issue : #31, #34, #36, #50
* Contributors: Leon

3.4.1 (2016-08-22)
-----------
* added ROS package folder for ROS users
* modified original header files for ROS package
* Contributors: Leon

3.4.0 (2016-08-12)
-----------
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
