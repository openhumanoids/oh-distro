add_definitions(-g)

pods_install_headers(Client.h DESTINATION ViconDataStreamSDK_CPP)


add_executable(myvicon vicon.cpp)
target_link_libraries(myvicon lcm ViconDataStreamSDK_CPP)
pods_use_pkg_config_packages(myvicon lcm lcmtypes_vicon-drc)
pods_install_executables(myvicon)


add_executable(vicon_driver ViconHolodeckDriver.cpp)
target_link_libraries(vicon_driver lcm ViconDataStreamSDK_CPP)
pods_use_pkg_config_packages(vicon_driver lcm lcmtypes_vicon-drc)
pods_install_executables(vicon_driver)

# Buggy version:, need to be changed to newer types
#add_executable(vicon_testreceive testreceive.c)
#target_link_libraries(vicon_testreceive lcm ViconDataStreamSDK_CPP)
#pods_use_pkg_config_packages(vicon_testreceive lcm lcmtypes_vicon-drc)
#pods_install_executables(vicon_testreceive)


# Previous Vicon System in Locomotion Lab:
#add_executable(vicon main.c structs.c vicon_client.c lcm_i.c byteconversion.c globalvar.c command_line.c)
#target_link_libraries(vicon lcm)
#pods_use_pkg_config_packages(vicon lcm lcmtypes_vicon-drc)
#pods_install_executables(vicon)

#add_executable(testmyvicon testmyvicon.cpp)
#target_link_libraries(testmyvicon lcm ViconDataStreamSDK_CPP)
#pods_use_pkg_config_packages(testmyvicon lcm lcmtypes_vicon-drc)
#pods_install_executables(testmyvicon)

#add_executable(drcpublish testpublish.c)
#target_link_libraries(drcpublish lcm)
#pods_use_pkg_config_packages(drcpublish lcm lcmtypes_vicon-drc)
#pods_install_executables(drcpublish)

#add_executable(drcreceive testreceive.c)
#target_link_libraries(drcreceive lcm)
#pods_use_pkg_config_packages(drcreceive lcm lcmtypes_vicon-drc)
#pods_install_executables(drcreceive)

#add_executable(testpublish_withmodel testpublish_withmodel.c)
#target_link_libraries(testpublish_withmodel lcm)
#pods_use_pkg_config_packages(testpublish_withmodel lcm lcmtypes_vicon-drc)
#pods_install_executables(testpublish_withmodel)

#add_executable(testpublish_angles testpublish_angles.c)
#target_link_libraries(testpublish_angles lcm m)
#pods_use_pkg_config_packages(testpublish_angles lcm lcmtypes_vicon-drc)
#pods_install_executables(testpublish_angles)
