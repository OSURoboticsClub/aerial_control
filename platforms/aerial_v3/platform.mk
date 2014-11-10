# Imported source files and paths
include $(CHIBIOS)/boards/ST_STM32F4_DISCOVERY/board.mk
include $(CHIBIOS)/os/hal/platforms/STM32F4xx/platform.mk
include $(CHIBIOS)/os/ports/GCC/ARMCMx/STM32F4xx/port.mk

# Define linker script file
LDSCRIPT= $(PORTLD)/STM32F405xG.ld

# Include directory for mcuconf.h
CONFINC = platforms/aerial_v3

# Sources for config
CPPSRC += $(wildcard platforms/aerial_v3/*.cpp)
