##################################################
# PROJECT: DynamixelSDK - ROBOTIS CO., Ltd.
##################################################

#---------------------------------------------------------------------
# C COMPILER, COMPILER FLAGS, AND TARGET PROGRAM NAME
#---------------------------------------------------------------------
DIR_DXL     = ../..
DIR_OBJS    = ./.objects

INSTALL_ROOT = /usr/local

MAJ_VERSION = 2
MIN_VERSION = 0
REV_VERSION = 0

TARGET      = libdxl_x64_c.so
TARGET1     = $(TARGET).$(MAJ_VERSION)
TARGET2     = $(TARGET).$(MAJ_VERSION).$(MIN_VERSION)
TARGET3     = $(TARGET).$(MAJ_VERSION).$(MIN_VERSION).$(REV_VERSION)

CHK_DIR_EXISTS = test -d
PRINT       = echo
STRIP       = strip
AR          = ar
ARFLAGS     = cr
LD          = g++
LDFLAGS     = -shared -fPIC $(FORMAT)#-Wl,-soname,dxl
LD_CONFIG   = ldconfig
CP          = cp
CP_ALL      = cp -r
RM          = rm
RM_ALL      = rm -rf
SYMLINK     = ln -s
MKDIR       = mkdir
CC          = gcc
CX          = g++
CCFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c $(INCLUDES) $(FORMAT) -fPIC -g
CXFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c $(INCLUDES) $(FORMAT) -fPIC -g
FORMAT     = -m64
INCLUDES    += -I$(DIR_DXL)/include/dynamixel_sdk

#---------------------------------------------------------------------
# Required external libraries
#---------------------------------------------------------------------
LIBRARIES  += -lrt

#---------------------------------------------------------------------
# SDK Files
#---------------------------------------------------------------------
SOURCES  = src/dynamixel_sdk/group_bulk_read.c \
           src/dynamixel_sdk/group_bulk_write.c \
           src/dynamixel_sdk/group_sync_read.c \
           src/dynamixel_sdk/group_sync_write.c \
           src/dynamixel_sdk/packet_handler.c \
           src/dynamixel_sdk/port_handler.c \
           src/dynamixel_sdk/protocol1_packet_handler.c \
           src/dynamixel_sdk/protocol2_packet_handler.c \
           src/dynamixel_sdk_linux/port_handler_linux.c \


OBJECTS=$(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))


#---------------------------------------------------------------------
# COMPILING RULES
#---------------------------------------------------------------------
$(TARGET): makedirs $(OBJECTS)
	$(LD) $(LDFLAGS) -o ./$(TARGET) $(OBJECTS) $(LIBRARIES)

makedirs:
	mkdir -p $(DIR_OBJS)/

clean:
	rm -f $(OBJECTS) ./$(TARGET)

install: $(TARGET)
    # copy the binaries into the lib directory
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/lib || $(MKDIR) $(INSTALL_ROOT)/lib
	-$(CP) "./$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET)"
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET1)"
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET2)"
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET3)"

    # copy the headers into the include directory
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/include/dynamixel_sdk || $(MKDIR) $(INSTALL_ROOT)/include/dynamixel_sdk
	$(CP_ALL) $(DIR_DXL)/include/dynamixel_sdk/* $(INSTALL_ROOT)/include/dynamixel_sdk/

	$(LD_CONFIG)

uninstall:
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET1)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET2)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET3)

	$(RM) $(INSTALL_ROOT)/include/dynamixel_sdk/dynamixel_sdk.h
	$(RM_ALL) $(INSTALL_ROOT)/include/dynamixel_sdk/*

reinstall: uninstall install


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
