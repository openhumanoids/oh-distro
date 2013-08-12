set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_C_FLAGS "-O2 -std=c99 -DF_CPU=8000000 -Wall -mmcu=atmega328p")
set(CMAKE_C_LINK_FLAGS "-Wl,-Map,psu.map")
add_executable(psu.elf main.c)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/psu.hex
                   COMMAND avr-objcopy -O ihex -R .eeprom ${CMAKE_CURRENT_BINARY_DIR}/psu.elf ${CMAKE_CURRENT_BINARY_DIR}/psu.hex
                   DEPENDS psu.elf)
add_custom_target(do_psu.hex ALL DEPENDS psu.hex)

add_custom_target(psu-program
                  COMMAND avrdude -p m328p -c avrispmkII -P usb -U flash:w:${CMAKE_CURRENT_BINARY_DIR}/psu.hex
                 DEPENDS psu.hex)
add_custom_target(psu-write-fuse
                  COMMAND avrdude -p m328p -c avrispmkII -P usb -U lfuse:w:0xE2:m -v)
