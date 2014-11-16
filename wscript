APPNAME = 'osuar_control'
VERSION = '0.0.1'

PORTSRC = [
    'ChibiOS-RT/os/ports/GCC/ARMCMx/crt0.c',
    'ChibiOS-RT/os/ports/GCC/ARMCMx/STM32F3xx/vectors.c',
    'ChibiOS-RT/os/ports/GCC/ARMCMx/chcore.c',
    'ChibiOS-RT/os/ports/GCC/ARMCMx/chcore_v7m.c',
    'ChibiOS-RT/os/ports/common/ARMCMx/nvic.c'
]
KERNSRC = [
    'ChibiOS-RT/os/kernel/src/chsys.c',
    'ChibiOS-RT/os/kernel/src/chdebug.c',
    'ChibiOS-RT/os/kernel/src/chlists.c',
    'ChibiOS-RT/os/kernel/src/chvt.c',
    'ChibiOS-RT/os/kernel/src/chschd.c',
    'ChibiOS-RT/os/kernel/src/chthreads.c',
    'ChibiOS-RT/os/kernel/src/chdynamic.c',
    'ChibiOS-RT/os/kernel/src/chregistry.c',
    'ChibiOS-RT/os/kernel/src/chsem.c',
    'ChibiOS-RT/os/kernel/src/chmtx.c',
    'ChibiOS-RT/os/kernel/src/chcond.c',
    'ChibiOS-RT/os/kernel/src/chevents.c',
    'ChibiOS-RT/os/kernel/src/chmsg.c',
    'ChibiOS-RT/os/kernel/src/chmboxes.c',
    'ChibiOS-RT/os/kernel/src/chqueues.c',
    'ChibiOS-RT/os/kernel/src/chmemcore.c',
    'ChibiOS-RT/os/kernel/src/chheap.c',
    'ChibiOS-RT/os/kernel/src/chmempools.c'
]
HALSRC = [
    'ChibiOS-RT/os/hal/src/hal.c',
    'ChibiOS-RT/os/hal/src/adc.c',
    'ChibiOS-RT/os/hal/src/can.c',
    'ChibiOS-RT/os/hal/src/ext.c',
    'ChibiOS-RT/os/hal/src/gpt.c',
    'ChibiOS-RT/os/hal/src/i2c.c',
    'ChibiOS-RT/os/hal/src/icu.c',
    'ChibiOS-RT/os/hal/src/mac.c',
    'ChibiOS-RT/os/hal/src/mmc_spi.c',
    'ChibiOS-RT/os/hal/src/mmcsd.c',
    'ChibiOS-RT/os/hal/src/pal.c',
    'ChibiOS-RT/os/hal/src/pwm.c',
    'ChibiOS-RT/os/hal/src/rtc.c',
    'ChibiOS-RT/os/hal/src/sdc.c',
    'ChibiOS-RT/os/hal/src/serial.c',
    'ChibiOS-RT/os/hal/src/serial_usb.c',
    'ChibiOS-RT/os/hal/src/spi.c',
    'ChibiOS-RT/os/hal/src/tm.c',
    'ChibiOS-RT/os/hal/src/uart.c',
]
PLATFORMSRC = [
    'ChibiOS-RT/os/hal/platforms/STM32F30x/stm32_dma.c',
    'ChibiOS-RT/os/hal/platforms/STM32F30x/hal_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32F30x/adc_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32F30x/ext_lld_isr.c',
    'ChibiOS-RT/os/hal/platforms/STM32/can_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32/ext_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32/GPIOv2/pal_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32/I2Cv2/i2c_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32/RTCv2/rtc_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32/SPIv2/spi_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32/TIMv1/gpt_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32/TIMv1/icu_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32/TIMv1/pwm_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32/USARTv2/serial_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32/USARTv2/uart_lld.c',
    'ChibiOS-RT/os/hal/platforms/STM32/USBv1/usb_lld.c'
]
CPPWRAPPERSRC = ['ChibiOS-RT/os/various/cpp_wrappers/ch.cpp']

PORTINC = [
    'ChibiOS-RT/os/ports/common/ARMCMx/CMSIS/include',
    'ChibiOS-RT/os/ports/common/ARMCMx',
    'ChibiOS-RT/os/ports/GCC/ARMCMx',
    'ChibiOS-RT/os/ports/GCC/ARMCMx/STM32F3xx'
]
KERNINC = ['ChibiOS-RT/os/kernel/include']
HALINC = ['ChibiOS-RT/os/hal/include']
PLATFORMINC = [
    'ChibiOS-RT/os/hal/platforms/STM32F30x',
    'ChibiOS-RT/os/hal/platforms/STM32',
    'ChibiOS-RT/os/hal/platforms/STM32/GPIOv2',
    'ChibiOS-RT/os/hal/platforms/STM32/I2Cv2',
    'ChibiOS-RT/os/hal/platforms/STM32/RTCv2',
    'ChibiOS-RT/os/hal/platforms/STM32/SPIv2',
    'ChibiOS-RT/os/hal/platforms/STM32/TIMv1',
    'ChibiOS-RT/os/hal/platforms/STM32/USARTv2',
    'ChibiOS-RT/os/hal/platforms/STM32/USB'
]
CPPWRAPPERINC = ['ChibiOS-RT/os/various/cpp_wrappers']

CHIBIOS_INC = PORTINC + KERNINC + HALINC + PLATFORMINC + CPPWRAPPERINC + ['include', 'platforms/stm32f3discovery', 'units/apollo']
CHIBIOS_SRC = PORTSRC + KERNSRC + HALSRC + PLATFORMSRC + CPPWRAPPERSRC

# PORTLD  = ${CHIBIOS}/os/ports/GCC/ARMCMx/STM32F3xx/ld
# LDSCRIPT = $(PORTLD)/STM32F303xC.ld

top = '.'
out = 'build'

def options(ctx):
    ctx.add_option('--unit', action='store', default=False)
    ctx.load('compiler_c compiler_cxx')

def configure(ctx):
    ctx.find_program('objcopy', var='OBJCOPY', mandatory=True)
    ctx.load('compiler_c compiler_cxx')

    ctx.env.CHIBIOS_INC = CHIBIOS_INC

    ctx.recurse('platforms/stm32f3discovery')

    # Flags shared between CFLAGS, CXXFLAGS, and LINKFLAGS
    BASEFLAGS = [
        '-O2',
        '-mcpu=cortex-m4',
        '-mfloat-abi=hard',
        '-mfpu=fpv4-sp-d16',
        '-mthumb',
        '-mno-thumb-interwork',
        '-mthumb',
        '-fomit-frame-pointer', '-falign-functions=16',
        '-ffunction-sections', '-fdata-sections', '-fno-common',
        '-fsingle-precision-constant',
    ]

    ctx.env.append_value('CFLAGS', BASEFLAGS)
    ctx.env.append_value('CFLAGS', [
        '-DCORTEX_USE_FPU=TRUE',
        '-DCHPRINTF_USE_FLOAT=1',
        '-DEIGEN_NO_DEBUG=1',
        '-DTHUMB_PRESENT',
        '-DTHUMB_NO_INTERWORKING'
    ])

    ctx.env.append_value('CXXFLAGS', ctx.env.CFLAGS)
    ctx.env.append_value('CXXFLAGS', [
        '-std=c++11',
        '-fno-rtti',
        '-fno-exceptions'
    ])

    ctx.env.append_value('LINKFLAGS', BASEFLAGS)
    ctx.env.append_value('LINKFLAGS', [
        '-nostartfiles',
        '-lm',
    ])
    ctx.env.append_value('LINKFLAGS', [
        '-Wl,-Map=../build/osuar_control.map,--cref,--no-warn-mismatch,--library-path=ChibiOS-RT/os/ports/GCC/ARMCMx,--script=${LDSCRIPT},--gc-sections'
    ])

def build(ctx):
    # Build the ChibiOS library statically
    ctx.stlib(
        source=CHIBIOS_SRC + \
            ['ChibiOS-RT/os/various/chprintf.c',
             'ChibiOS-RT/os/various/memstreams.c'],
        target='chibios',
        includes=CHIBIOS_INC + ['ChibiOS-RT/os/various/', 'src']
    )

    # Build the program binary
    ctx.program(
        source=ctx.path.ant_glob('src/**/*.cpp') +
               ctx.path.ant_glob('platforms/stm32f3discovery/*.cpp'),
        target='osuar_control.elf',
        use='chibios',
        includes=CHIBIOS_INC + ['ChibiOS-RT/os/various/', 'src']
    )

    # Create the final binary file
    ctx(
        rule='${OBJCOPY} -O binary ${SRC} ${TGT}',
        source=ctx.path.get_bld().make_node('osuar_control.elf'),
        target=ctx.path.get_bld().make_node('osuar_control.bin')
    )
