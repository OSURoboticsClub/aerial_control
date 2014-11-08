CHIBIOS = ChibiOS-RT
include platforms/rocket_v0.1/platform.mk
include platforms/ch.mk

# Include directory for unit_config.hpp
CONFINC += units/hera

hera: all
