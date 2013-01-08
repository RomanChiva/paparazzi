# Hey Emacs, this is a -*- makefile -*-
#
# ardrone2.makefile
#
# http://paparazzi.enac.fr/wiki/AR.Drone_2_-_Specifications
#

BOARD=ardrone
BOARD_VERSION=2
BOARD_CFG=\"boards/$(BOARD)$(BOARD_VERSION).h\"

ARCH=omap_ardrone2
$(TARGET).ARCHDIR = $(ARCH)

# -----------------------------------------------------------------------
USER=foobar
HOST=ardrone2
SUB_DIR=bin
FTP_DIR=/data/video
TARGET_DIR=$(FTP_DIR)/$(SUB_DIR)
# -----------------------------------------------------------------------

# Do we need to disable modem? We don't have a modem.
#ifndef MODEM_PORT
#MODEM_PORT=UART0
#endif

#ifndef MODEM_BAUD
#MODEM_BAUD=B57600
#endif

# The GPS sensor is connected trough USB, we have to fix this
ifndef GPS_PORT
GPS_PORT=UART1
endif

ifndef GPS_BAUD
GPS_BAUD=B57600
endif

# This is a (temporary) fix for uart_arch.c to compile with a device name
#$(TARGET).CFLAGS += -DUART0_DEV=\"/dev/ttyO3\"
$(TARGET).CFLAGS += -DUART1_DEV=\"/dev/ttyUSB0\"
# -----------------------------------------------------------------------
