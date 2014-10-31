# Imported source files and paths
include $(CHIBIOS)/boards/ST_STM32F3_DISCOVERY/board.mk
include $(CHIBIOS)/os/hal/platforms/STM32F30x/platform.mk
include $(CHIBIOS)/os/ports/GCC/ARMCMx/STM32F3xx/port.mk

# Define linker script file
LDSCRIPT = $(PORTLD)/STM32F303xC.ld

# Include directory for mcuconf.h
CONFINC = platforms/stm32f3discovery
