include platforms/stm32f3discovery/platform.mk
include platforms/ch.mk

# Include directory for unit_config.hpp
CONFINC += units/apollo

apollo: all
