add_executable(f2-std.elf main.c tactile.c io.c comms.c startup.c pins.c imu.c)
target_link_libraries(f2-std.elf common_sam3s)
