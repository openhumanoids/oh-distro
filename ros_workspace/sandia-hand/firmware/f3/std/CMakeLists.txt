add_executable(f3-std.elf main.c tactile.c io.c comms.c startup.c pins.c imu.c)
target_link_libraries(f3-std.elf common_sam3s)
