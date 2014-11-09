APPNAME = 'osuar_control'
VERSION = '0.0.1'

CHIBIOS_INCLUDES = [
    'ChibiOS-RT/os/ports/common/ARMCMx/CMSIS/include',
    'ChibiOS-RT/os/ports/common/ARMCMx',
    'ChibiOS-RT/os/ports/GCC/ARMCMx',
    'ChibiOS-RT/os/ports/GCC/ARMCMx/STM32F3xx',
    'ChibiOS-RT/os/kernel/include',
    'ChibiOS-RT/test',
    'ChibiOS-RT/os/hal/include',
    'ChibiOS-RT/os/hal/platforms/STM32F30x',
    'ChibiOS-RT/os/hal/platforms/STM32',
    'ChibiOS-RT/os/hal/platforms/STM32/GPIOv2',
    'ChibiOS-RT/os/hal/platforms/STM32/I2Cv2',
    'ChibiOS-RT/os/hal/platforms/STM32/RTCv2',
    'ChibiOS-RT/os/hal/platforms/STM32/SPIv2',
    'ChibiOS-RT/os/hal/platforms/STM32/TIMv1',
    'ChibiOS-RT/os/hal/platforms/STM32/USARTv2',
    'ChibiOS-RT/os/hal/platforms/STM32/USBv1',
    'ChibiOS-RT/boards/ST_STM32F3_DISCOVERY',
    'platforms/stm32f3discovery',
    'units/apollo',
    'ChibiOS-RT/os/various',
    'ChibiOS-RT/os/various/cpp_wrappers',
    'include',
    'src'
]

top = '.'
out = 'build'

def options(ctx):
    ctx.add_option('--unit', action='store', default=False)
    ctx.load('compiler_c compiler_cxx')

def configure(ctx):
    ctx.env.CFLAGS = ['-mcpu=cortex-m4', '-mfloat-abi=hard',
            '-mfpu=fpv4-sp-d16', '-mthumb',
            '-fomit-frame-pointer', '-falign-functions=16', '-lm', '-ffunction-sections', '-fdata-sections', '-fno-common',
            '-DCORTEX_USE_FPU=TRUE', '-DCHPRINTF_USE_FLOAT=1', '-DEIGEN_NO_DEBUG=1', '-DTHUMB_PRESENT', '-mno-thumb-interwork', '-DTHUMB_NO_INTERWORKING',
            '']
    ctx.env.CXXFLAGS = ['-mcpu=cortex-m4', '-mfloat-abi=hard',
            '-mfpu=fpv4-sp-d16', '-mthumb',
            '-fomit-frame-pointer', '-falign-functions=16', '-lm', '-ffunction-sections', '-fdata-sections', '-fno-common',
            '-DCORTEX_USE_FPU=TRUE', '-DCHPRINTF_USE_FLOAT=1', '-DEIGEN_NO_DEBUG=1', '-DTHUMB_PRESENT', '-mno-thumb-interwork', '-DTHUMB_NO_INTERWORKING',
            '-std=c++11']
    ctx.load('compiler_c compiler_cxx')

def build(ctx):
    ctx.objects(
        source=ctx.path.ant_glob('ChibiOS-RT/os/hal/src/*.c'),
        target='objs1',
        includes=CHIBIOS_INCLUDES
    )
    ctx.objects(
        source=ctx.path.ant_glob('ChibiOS-RT/os/kernel/src/*.c'),
        target='objs2',
        includes=CHIBIOS_INCLUDES
    )
    ctx.objects(
        source=ctx.path.ant_glob('ChibiOS-RT/test/*.c'),
        target='objs_test',
        includes=CHIBIOS_INCLUDES
    )
    ctx.objects(
        source=ctx.path.ant_glob('ChibiOS-RT/os/various/cpp_wrappers/*.cpp'),
        target='objs3',
        includes=CHIBIOS_INCLUDES
    )
    ctx.objects(
        source=ctx.path.ant_glob('src/**/*.cpp'),
        target='objs4',
        includes=CHIBIOS_INCLUDES
    )
    ctx.program(
        source='src/main.cpp',
        target='out',
        use='objs1 objs2 objs_test objs3 objs4',
        includes=CHIBIOS_INCLUDES
    )
