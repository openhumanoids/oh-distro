cmake_minimum_required(VERSION 2.8.3)
project(lwr_defs)
find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS xacro)

foreach(dir calibration defs meshes robots)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
