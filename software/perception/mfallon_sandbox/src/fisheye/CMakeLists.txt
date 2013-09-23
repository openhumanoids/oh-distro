# this is not correctly calculating the lidar transforms, out of date:
add_executable(mf-fisheye fisheye.cpp)
pods_use_pkg_config_packages(mf-fisheye lcm image_io_utils
              bot2-frames bot2-lcmgl-client eigen3)
pods_install_executables(mf-fisheye)



find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
set(REQUIRED_LIBS ${VTK_IO_TARGET_LINK_LIBRARIES} ${GLEW_LIBRARIES}    
       ${GLUT_LIBRARIES} vtkCommon vtkIO vtkHybrid)
set(PCL_LIBRARIES  pcl_io-1.6 )

add_executable(mf-fisheye-meshproject fisheye-meshproject.cpp)
pods_use_pkg_config_packages(mf-fisheye-meshproject lcm image_io_utils
   pointcloud_vis ${PCL_LIBRARIES} opencv rgbd_primitives
              bot2-frames bot2-lcmgl-client eigen3)
              
target_link_libraries (mf-fisheye-meshproject boost_system 
           ${REQUIRED_LIBS})              
pods_install_executables(mf-fisheye-meshproject)
