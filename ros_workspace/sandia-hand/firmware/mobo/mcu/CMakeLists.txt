set(CMAKE_C_LINK_FLAGS ${SAM3X_APP_LINK_FLAGS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../ros/sandia_hand_driver/include)
include_directories(${PROJECT_SOURCE_DIR}/common_sam3x)
add_definitions(-DF_CPU=64000000) 
add_executable(mobo-mcu.elf main.c startup.c power.c udp_rx.c finger.c cam.c control.c)
add_custom_command(OUTPUT mobo-mcu.bin
                   COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-objcopy -O binary ${CMAKE_CURRENT_BINARY_DIR}/mobo-mcu.elf ${CMAKE_CURRENT_BINARY_DIR}/mobo-mcu.bin
                   DEPENDS mobo-mcu.elf)
add_custom_target(do_mobo-mcu.bin ALL DEPENDS mobo-mcu.bin)

#set_target_properties(mobo_mcu.elf PROPERTIES LINK_FLAGS "-T\"${CMAKE_CURRENT_SOURCE_DIR}/../common_sam3x/sam3x8e.ld\"")
target_link_libraries(mobo-mcu.elf common_sam3x c gcc)

set(OPENOCD_INIT ${OPENOCD} -f ${PROJECT_SOURCE_DIR}/openocd/olimex-arm-usb-tiny-h.cfg -f ${PROJECT_SOURCE_DIR}/mobo/openocd.cfg -c init -c halt)

add_custom_target(mobo-mcu-size
                  COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-size -A -x ${CMAKE_CURRENT_BINARY_DIR}/mobo-mcu.elf
                  DEPENDS mobo-mcu.elf)

add_custom_target(mobo-mcu-names
                  COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-nm -n ${CMAKE_CURRENT_BINARY_DIR}/mobo-mcu.elf >mobo-mcu.elf.nm.txt && vim mobo-mcu.elf.nm.txt
                  DEPENDS mobo-mcu.elf)

add_custom_target(mobo-mcu-disassemble
                  COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-objdump -S -d ${CMAKE_CURRENT_BINARY_DIR}/mobo-mcu.elf >disassembly.txt && vim disassembly.txt
                  DEPENDS mobo-mcu.elf)
          
add_custom_target(mobo-mcu-program
                  COMMAND ${OPENOCD_INIT} -c "flash banks" -c "flash list" -c "flash write_image ${CMAKE_CURRENT_BINARY_DIR}/mobo-mcu.elf" -c reset -c shutdown
                  DEPENDS mobo-mcu.elf)

add_custom_target(mobo-mcu-reset
                  COMMAND ${OPENOCD_INIT} -c reset -c shutdown)

add_custom_target(mobo-mcu-set_boot_vector
                  COMMAND ${OPENOCD_INIT} -c "at91sam3 gpnvm set 1" -c shutdown)

add_custom_target(mobo-query 
                  COMMAND ${OPENOCD_INIT} -c scan_chain -c targets sam3.cpu -c "at91sam3 info" -c reset -c shutdown)
 
add_custom_target(mobo-gdb-server
                  COMMAND ${OPENOCD_INIT} -c halt)

add_custom_target(mobo-gdb
                  COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-gdb ${CMAKE_CURRENT_BINARY_DIR}/mobo-mcu.elf)


