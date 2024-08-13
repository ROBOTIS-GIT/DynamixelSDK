##################################################
# PROJECT: DynamixelSDK - ROBOTIS CO., Ltd.
##################################################

#
# Makefile for Windows DLL and gcc (MSYS2) in ordinary Command Prompt
#

#---------------------------------------------------------------------
# C++ COMPILER, COMPILER FLAGS, AND TARGET PROGRAM NAME
#---------------------------------------------------------------------
DIR_DXL     = ../..
DIR_OBJS    = .objects
DIR_OUTPUT	= output

MAJ_VERSION = 2
MIN_VERSION = 0
REV_VERSION = 0

TARGET      = $(DIR_OUTPUT)/libdxl_x64_cpp.dll
TARGET1     = $(TARGET).$(MAJ_VERSION)
TARGET2     = $(TARGET).$(MAJ_VERSION).$(MIN_VERSION)
TARGET3     = $(TARGET).$(MAJ_VERSION).$(MIN_VERSION).$(REV_VERSION)

PRINT       = echo
STRIP       = strip
AR          = ar
ARFLAGS     = cr
LD          = g++
LDFLAGS     = -shared -fPIC $(FORMAT)#-Wl,-soname,dxl
MKDIR       = mkdir
CC          = gcc
CX          = g++
CCFLAGS     = -O2 -O3 -D_GNU_SOURCE -Wall -c $(INCLUDES) $(FORMAT) -fPIC -g
CXFLAGS     = -O2 -O3 -D_GNU_SOURCE -Wall -c $(INCLUDES) $(FORMAT) -fPIC -g
FORMAT      = -m64
INCLUDES    += -I$(DIR_DXL)/include/dynamixel_sdk

#---------------------------------------------------------------------
# Required external libraries
#---------------------------------------------------------------------
#LIBRARIES  += -lrt

#---------------------------------------------------------------------
# SDK Files
#---------------------------------------------------------------------
SOURCES  = src/dynamixel_sdk/group_bulk_read.cpp \
           src/dynamixel_sdk/group_bulk_write.cpp \
           src/dynamixel_sdk/group_sync_read.cpp \
           src/dynamixel_sdk/group_sync_write.cpp \
           src/dynamixel_sdk/group_fast_bulk_read.cpp \
           src/dynamixel_sdk/group_fast_sync_read.cpp \
           src/dynamixel_sdk/group_handler.cpp \
           src/dynamixel_sdk/packet_handler.cpp \
           src/dynamixel_sdk/port_handler.cpp \
           src/dynamixel_sdk/protocol1_packet_handler.cpp \
           src/dynamixel_sdk/protocol2_packet_handler.cpp \
           src/dynamixel_sdk/port_handler_windows.cpp \


OBJECTS=$(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))


#---------------------------------------------------------------------
# COMPILING RULES
#---------------------------------------------------------------------
$(TARGET): $(DIR_OBJS) $(DIR_OUTPUT) $(OBJECTS)
	$(LD) $(LDFLAGS) -o $(TARGET) $(OBJECTS) $(LIBRARIES)

$(DIR_OBJS):
	$(MKDIR) $(DIR_OBJS)

$(DIR_OUTPUT):
	$(MKDIR) $(DIR_OUTPUT)

clean:
	rmdir /s /q $(DIR_OBJS) $(DIR_OUTPUT)


#---------------------------------------------------------------------
# Make rules for all .c and .cpp files in each directory
#---------------------------------------------------------------------

$(DIR_OBJS)/%.o: $(DIR_DXL)/src/dynamixel_sdk/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: $(DIR_DXL)/src/dynamixel_sdk/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: $(DIR_DXL)/src/dynamixel_sdk/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: $(DIR_DXL)/src/dynamixel_sdk/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

#---------------------------------------------------------------------
# END OF MAKEFILE
#---------------------------------------------------------------------
