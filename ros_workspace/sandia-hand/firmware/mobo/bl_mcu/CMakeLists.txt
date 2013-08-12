set(CMAKE_C_LINK_FLAGS "-g -mthumb -mcpu=cortex-m3 -T\"${PROJECT_SOURCE_DIR}/common_sam3x/sam3x8e_bl.ld\" -Wl,-Map=memory.map -Wl,--cref --entry=reset_vector -Wl,--no-gc-sections -Wl,--warn-common -Wl,--unresolved-symbols=report-all -Wl,--no-undefined -Wl,--start-group -lc -lgcc -Wl,--end-group -Wl,--section-start=.bl_hw_version=0x87ffc -Wl,--section-start=.bl_hand_ip=0x87ff8 -Wl,--section-start=.bl_master_ip=0x87ff4 -Wl,--section-start=.bl_hand_mac=0x87fec")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../ros/sandia_hand_driver/include)
include_directories(${PROJECT_SOURCE_DIR}/common_sam3x)
add_definitions(-DF_CPU=64000000) 
set(OPENOCD_INIT ${OPENOCD} -f ${PROJECT_SOURCE_DIR}/openocd/olimex-arm-usb-tiny-h.cfg -f ${PROJECT_SOURCE_DIR}/mobo/openocd.cfg -c init -c halt)

set(mobo_bootloaders mobo-mcu-bl-left mobo-mcu-bl-right)
foreach(bl ${mobo_bootloaders})
  set(exe_name ${bl})
  set(elf_name ${exe_name}.elf)
  set(bin_name ${exe_name}.bin)
  add_executable(${elf_name} main.c startup.c udp_rx.c boot.c ${bl}.c)
  add_custom_command(OUTPUT ${bin_name}
                     COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-objcopy -O binary ${CMAKE_CURRENT_BINARY_DIR}/${elf_name} ${CMAKE_CURRENT_BINARY_DIR}/${bin_name}
                     DEPENDS ${elf_name})
  add_custom_target(do_${bin_name} ALL DEPENDS ${bin_name})
  target_link_libraries(${elf_name} common_sam3x c gcc)

  add_custom_target(${bl}-size
                    COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-size -A -x ${CMAKE_CURRENT_BINARY_DIR}/${elf_name}
                    DEPENDS ${elf_name})

  add_custom_target(${bl}-names
                    COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-nm -n ${CMAKE_CURRENT_BINARY_DIR}/${elf_name} >${elf_name}.nm.txt && vim ${elf_name}.nm.txt
                    DEPENDS ${elf_name})

  add_custom_target(${bl}-disassemble
                    COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-objdump -S -d ${CMAKE_CURRENT_BINARY_DIR}/${elf_name} >disassembly.txt && vim disassembly.txt
                    DEPENDS ${elf_name})

  add_custom_target(${bl}-program
                    COMMAND ${OPENOCD_INIT} -c "flash banks" -c "flash list" -c "flash write_image ${CMAKE_CURRENT_BINARY_DIR}/${elf_name}" -c reset -c shutdown
                    DEPENDS ${elf_name})

  # evidently openocd sam3x driver doesn't seem to let this happen 
  # since it wants to auto-erase when programming?
  #add_custom_target(${bl}-erase
  #                  COMMAND ${OPENOCD_INIT} -c "flash banks" -c "flash list" -c "flash erase_address 0x80000 0x20000" -c shutdown)

  add_custom_target(${bl}-reset
                    COMMAND ${OPENOCD_INIT} -c reset -c shutdown)

  add_custom_target(${bl}-bork_boot_vector
                    COMMAND ${OPENOCD_INIT} -c "at91sam3 gpnvm clear 1" -c shutdown)

  add_custom_target(${bl}-set_boot_vector
                    COMMAND ${OPENOCD_INIT} -c "at91sam3 gpnvm set 1" -c shutdown)

  add_custom_target(${bl}-query
                    COMMAND ${OPENOCD_INIT} -c scan_chain -c targets sam3.cpu -c "at91sam3 info" -c reset -c shutdown)
 
  add_custom_target(${bl}-gdb-server
                    COMMAND ${OPENOCD_INIT} -c halt)

  add_custom_target(${bl}-gdb
                    COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-gdb ${CMAKE_CURRENT_BINARY_DIR}/${elf_name})
 
endforeach()
