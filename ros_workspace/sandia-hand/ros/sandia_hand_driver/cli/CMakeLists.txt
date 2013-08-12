rosbuild_add_executable(sandia_hand_cli sandia_hand_cli.cpp)
target_link_libraries(sandia_hand_cli sandia_hand)
install(TARGETS sandia_hand_cli DESTINATION bin)

rosbuild_add_executable(loose_finger_cli loose_finger_cli.cpp)
target_link_libraries(loose_finger_cli sandia_hand)
install(TARGETS loose_finger_cli DESTINATION bin
        RENAME sandia_hand_loose_finger_cli)

rosbuild_add_executable(loose_palm_cli loose_palm_cli.cpp)
target_link_libraries(loose_palm_cli sandia_hand)
install(TARGETS loose_palm_cli DESTINATION bin
        RENAME sandia_hand_loose_palm_cli)

rosbuild_add_executable(sandia_hand_firmware_updater firmware_updater.cpp)
target_link_libraries(sandia_hand_firmware_updater sandia_hand)
install(TARGETS sandia_hand_firmware_updater DESTINATION bin)
