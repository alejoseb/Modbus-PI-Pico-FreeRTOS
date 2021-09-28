cmake_minimum_required(VERSION 3.12)

#include(pico_sdk_import.cmake)
# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()

include(pico_sdk_import.cmake)

add_subdirectory(pico-cpp)

project(test_modbus C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


pico_sdk_init()

add_executable(test_modbus
    src/main.cpp 
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(test_modbus pico_stdlib hardware_uart)

add_library(FreeRTOS STATIC 
FreeRTOS-Kernel/croutine.c
FreeRTOS-Kernel/event_groups.c
FreeRTOS-Kernel/list.c
FreeRTOS-Kernel/queue.c
FreeRTOS-Kernel/stream_buffer.c
FreeRTOS-Kernel/tasks.c
FreeRTOS-Kernel/timers.c 
FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c 
FreeRTOS-Kernel/portable/MemMang/heap_4.c 
src/FreeRTOS_Support_Files/IdleMemory.c)

target_include_directories(FreeRTOS PUBLIC FreeRTOS-Kernel/include include/ FreeRTOS-Kernel/portable/GCC/ARM_CM0 MODBUS-LIB/Inc)


add_library(Modbus STATIC MODBUS-LIB/Src/Modbus.c)
target_link_libraries(Modbus pico_stdlib hardware_uart)
target_include_directories(Modbus PUBLIC FreeRTOS-Kernel/include include/ FreeRTOS-Kernel/portable/GCC/ARM_CM0 MODBUS-LIB/Inc)


# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(test_modbus)
target_link_libraries(test_modbus FreeRTOS pico-cpp Modbus)
add_custom_command(TARGET test_modbus
        POST_BUILD 
        COMMAND arm-none-eabi-size test_modbus.elf
        )
# rest of your project