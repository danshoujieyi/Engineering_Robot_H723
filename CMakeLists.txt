#此文件从模板自动生成! 请勿更改!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.28)

# 指定交叉编译工具链
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 项目设置
project(CtrBoard-H7_ALL C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# 硬件浮点数FPU设置
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# -------------------- 关键修改：单精度 FPU 和 DSP 配置 --------------------
# 硬件浮点配置（单精度）
add_compile_definitions(
        ARM_MATH_CM7;                  # 指定 CMSIS-DSP 为 M7 内核
        ARM_MATH_MATRIX_CHECK;
        ARM_MATH_ROUNDING
)
add_compile_options(
        -mfloat-abi=hard
        -mfpu=fpv5-sp-d16             # 单精度 FPU（关键修改：sp 表示单精度）
        -mcpu=cortex-m7               # 指定 M7 架构
)
add_link_options(
        -mfloat-abi=hard
        -mfpu=fpv5-sp-d16
        -mcpu=cortex-m7
)

## 硬件浮点配置（双精度FPU），需要修改FreeRTOSConfig.h文件，configUSE_TASK_FPU_SUPPORT设置为2
#add_compile_definitions(
#        ARM_MATH_CM7                  # 指定CMSIS-DSP为M7内核
#        ARM_MATH_MATRIX_CHECK
#        ARM_MATH_ROUNDING
#)
#add_compile_options(
#        -mfloat-abi=hard
#        -mfpu=fpv5-d16                # M7双精度FPU
#        -mcpu=cortex-m7               # 指定M7架构
#)
#add_link_options(
#        -mfloat-abi=hard
#        -mfpu=fpv5-d16
#        -mcpu=cortex-m7
#)


# 软件模拟FPU浮点数运算
#add_compile_options(-mfloat-abi=soft)

# 通用编译选项
add_compile_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# 抑制-Wregister警告。-Wregister警告是 GCC 和 Clang 等编译器在使用 C++17 标准及以后版本时可能发出的警告。
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# 启用汇编预处理器
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# 构建类型配置
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

# 包含目录（根据实际路径调整）
include_directories(Core/Inc Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/CMSIS/Include
        # USB设备
        USB_DEVICE/App USB_DEVICE/Target
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

        Middlewares/ST/ARM/DSP/Inc # 矩阵运算库

        USER/Algorithm/Crc8_Crc16
        USER/Algorithm/Filter
        USER/Algorithm/KalmanFilter
        USER/Algorithm/KalmanFilterOne
        #USER/Algorithm/MahonyAHRS
        USER/Algorithm/MyType
        USER/Algorithm/PID
        USER/Algorithm/QuaternionEKF
        USER/Algorithm/Ramp
        USER/Algorithm/User_Lib

        USER/Module/Board_Device/Board_Bmi088
        USER/Module/Board_Device/Board_Passive_Buzzer
        USER/Module/Board_Device/Board_WS2812
        USER/Module/Board_Expansion_Device/Expansion_Board_LCD_169
        USER/Module/Data_Transfer_Display/VOFA
        USER/Module/Dwt
        USER/Module/Fdcan
        USER/Module/Motor/DJ_Motor_can
        USER/Module/Motor/DM_Motor_Fdcan
        USER/Module/Msg_Topics/Msg_Topics_FreeRTOS
        USER/Module/Pump
        USER/Module/Referee_System
        USER/Module/Remote_Control/FS_SBUS
        USER/Module/Remote_Control/Keyboard

        USER/Task/Algorithm
        USER/Task/Chassis
        USER/Task/Cmd
        USER/Task/DJmotor
        USER/Task/DMmotor
        USER/Task/Ins
        #USER/Task/OneNet_ESP8266
        USER/Task/Referee_System
        USER/Task/Robot_Config
        USER/Task/Transmission
        USER/Task/USART
)

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_PWR_LDO_SUPPLY)

# 源文件
file(GLOB_RECURSE SOURCES "Core/*.*" "Middlewares/*.*" "Drivers/*.*" "USB_DEVICE/*.*"
        "USER/Algorithm/Crc8_Crc16/*.*"
        "USER/Algorithm/Filter/*.*"
        "USER/Algorithm/KalmanFilter/*.*"
        "USER/Algorithm/KalmanFilterOne/*.*"
        #"USER/Algorithm/MahonyAHRS/*.*"
        "USER/Algorithm/MyType/*.*"
        "USER/Algorithm/PID/*.*"
        "USER/Algorithm/QuaternionEKF/*.*"
        "USER/Algorithm/Ramp/*.*"
        "USER/Algorithm/User_Lib/*.*"

        "USER/Module/Board_Device/Board_Bmi088/*.*"
        "USER/Module/Board_Device/Board_Passive_Buzzer/*.*"
        "USER/Module/Board_Device/Board_WS2812/*.*"
        "USER/Module/Board_Expansion_Device/Expansion_Board_LCD_169/*.*"
        "USER/Module/Data_Transfer_Display/VOFA/*.*"
        "USER/Module/Dwt/*.*"
        "USER/Module/Fdcan/*.*"
        "USER/Module/Motor/DJ_Motor_can/*.*"
        "USER/Module/Motor/DM_Motor_Fdcan/*.*"
        "USER/Module/Msg_Topics/Msg_Topics_FreeRTOS/*.*"
        "USER/Module/Pump/*.*"
        "USER/Module/Referee_System/*.*"
        "USER/Module/Remote_Control/FS_SBUS/*.*"
        "USER/Module/Remote_Control/Keyboard/*.*"

        "USER/Task/Algorithm/*.*"
        "USER/Task/Chassis/*.*"
        "USER/Task/Cmd/*.*"
        "USER/Task/DJmotor/*.*"
        "USER/Task/DMmotor/*.*"
        "USER/Task/Ins/*.*"
        #"USER/Task/OneNet_ESP8266/*.*"
        "USER/Task/Referee_System/*.*"
        "USER/Task/Robot_Config/*.*"
        "USER/Task/Transmission/*.*"
        "USER/Task/USART/*.*"

)

# 链接脚本
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H723VGTX_FLASH.ld)

# 链接选项
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

# 可执行文件
add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

# 链接DSP矩阵运算库,libarm_cortexM7lfsp_math单精度DSP库，
target_link_libraries(${PROJECT_NAME}.elf ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a)

# 生成Hex/Bin文件
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
