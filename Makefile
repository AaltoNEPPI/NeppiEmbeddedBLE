####
#### Sample Makefile for building applications with the RIOT OS
####
#### The example file system layout is:
#### ./application Makefile
#### ../../RIOT
####
WERROR ?= 0
# Set the name of your application:
APPLICATION = Neppitesti

# If no BOARD is found in the environment, use this default:
BOARD ?= nrf52dk

# If no PORT, which specifies the serial interface, is found in the
# environment, use this default:
PORT  ?= /dev/ttyACM0

# Uncomment this to disable optimizations for easier debugging:
CFLAGS_OPT ?= -O0

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../../../RIOT

# Uncomment this to enable scheduler statistics for ps:
#CFLAGS += -DSCHEDSTATISTICS

# If you want to use native with valgrind, you should recompile native
# with the target all-valgrind instead of all:
# make -B clean all-valgrind

# Uncomment this to enable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP = 1

# Change this to 0 to show compiler invocation lines by default:
# QUIET ?= 0

# Modules to include

#USEMODULE += shell
#USEMODULE += posix
USEMODULE += xtimer
USEMODULE += periph_uart
#USEMODULE += mpu9250
#USEMODULE += bme280
#USEMODULE += tcs34725
# Packages to include

USEPKG += nordic_softdevice_ble

# If your application is very simple and doesn't use modules that use
# messaging, it can be disabled to save some memory:

#DISABLE_MODULE += core_msg

#export INCLUDES += -Iapplication_include

# Specify custom dependencies for your application here ...
# APPDEPS = app_data.h config.h

include $(RIOTBASE)/Makefile.include

# ... and define them here (after including Makefile.include,
# otherwise you modify the standard target):
#proj_data.h: script.py data.tar.gz
#	./script.py