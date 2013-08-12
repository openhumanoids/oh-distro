include_directories(${PROJECT_SOURCE_DIR}/../ros/sandia_hand_driver/include/sandia_hand)
add_executable(palm-std.elf main.c startup.c comms.c imu.c io.c pins.c tactile.c state.c)
target_link_libraries(palm-std.elf common_sam3s)
