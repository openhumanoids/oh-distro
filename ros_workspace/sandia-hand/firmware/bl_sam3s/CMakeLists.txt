set(CMAKE_C_LINK_FLAGS "-g -mthumb -mcpu=cortex-m3 -T\"${CMAKE_CURRENT_SOURCE_DIR}/bl_sam3s.ld\" -Wl,-Map=memory.map -Wl,--cref --entry=_bootloader_start -Wl,--no-gc-sections -Wl,--warn-common -Wl,--unresolved-symbols=report-all -Wl,--no-undefined -nostdlib")
add_definitions(-Dsam3s4)
include_directories(${PROJECT_SOURCE_DIR}/common_sam3s)

set(bootloaders f2 f3 fmcb rpalm lpalm)
foreach(bl ${bootloaders})
  add_executable(bl_${bl}.elf bl_sam3s.c bl_sam3s_startup.S bl_${bl}.c)
  add_custom_target(${bl}-bl-program
                    COMMAND ${SAM3S_OPENOCD_INIT} -c "flash banks" -c "flash list" -c "flash write_image ${CMAKE_CURRENT_BINARY_DIR}/bl_${bl}.elf" -c reset -c shutdown
                    DEPENDS bl_${bl}.elf)
  add_custom_target(${bl}-bl-disassemble
                    COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-objdump -S -d ${CMAKE_CURRENT_BINARY_DIR}/bl_${bl}.elf | vim -
                    DEPENDS bl_${bl}.elf)
  add_custom_target(${bl}-bl-gdb-server COMMAND ${SAM3S_OPENOCD_INIT} -c halt)
  add_custom_target(${bl}-bl-gdb 
                    COMMAND ${ARM_GCC_ROOT}/bin/arm-none-eabi-gdb ${CMAKE_CURRENT_BINARY_DIR}/bl_${bl}.elf -x ${PROJECT_SOURCE_DIR}/openocd/gdb_init_commands)
  add_custom_target(${bl}-bl-gpnvm 
                    COMMAND ${SAM3S_OPENOCD_INIT} -c "at91sam3 gpnvm set 1" -c shutdown)
endforeach()

