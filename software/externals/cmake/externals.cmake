

if(NOT USE_SYSTEM_LCM)
  set(lcm_proj lcm)
  set(lcm_url https://github.com/lcm-proj/lcm/releases/download/v1.3.0/lcm-1.3.0.zip)
  set(lcm_download_hash 5d46a902fe04608809af3dc526909f9b)
  set(lcm_depends)
  set(lcm_external_args
    CONFIGURE_COMMAND ${source_prefix}/lcm/configure --prefix=${CMAKE_INSTALL_PREFIX}
    BUILD_IN_SOURCE 1
    )
endif()

set(bot_core_lcmtypes_url https://github.com/iamwolf/bot_core_lcmtypes.git)
set(bot_core_lcmtypes_revision c29cd6076d13ca2a3ecc23ffcbe28a0a1ab46314)
set(bot_core_lcmtypes_depends ${lcm_proj})

set(libbot_url https://github.com/openhumanoids/libbot.git)
set(libbot_revision ed4a76423f2a21594436490341f907710d3f78dd)
set(libbot_depends bot_core_lcmtypes ${lcm_proj})

set(Eigen_pod_url https://github.com/RobotLocomotion/eigen-pod.git)
set(Eigen_pod_revision ceba39500b89a77a8649b3e8b421b10a3d74d42b)
set(Eigen_pod_depends)

if(NOT USE_SYSTEM_OPENCV)
  set(opencv_proj opencv)
  set(opencv_url https://github.com/Itseez/opencv.git)
  set(opencv_revision 2.4.12.3)
  set(opencv_depends Eigen_pod)
  set(opencv_external_args
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${python_args}
      -DWITH_CUDA:BOOL=OFF
    )
endif()

set(flann_url https://github.com/mariusmuja/flann.git)
set(flann_revision 4969acc) # master from march 2015
set(flann_depends)
set(flann_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    -DBUILD_MATLAB_BINDINGS:BOOL=OFF
    -DBUILD_PYTHON_BINDINGS:BOOL=OFF
    -DBUILD_TESTS:BOOL=OFF
    -DBUILD_EXAMPLES:BOOL=OFF
  )

if(NOT USE_SYSTEM_PCL)
  set(pcl_proj pcl)
  set(pcl_url http://github.com/pointcloudlibrary/pcl.git)
  #set(pcl_revision pcl-1.7.2) # this version introduces some missing openni pkg-config file bug
  set(pcl_revision pcl-1.7.1)
  set(pcl_depends flann Eigen_pod)
  set(pcl_external_args
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
    )
endif()

set(octomap_url https://github.com/OctoMap/octomap.git)
set(octomap_revision 5b11e4c6bfd6dba54dd5bd1eea63354b6e18d835)
set(octomap_depends)
set(octomap_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
  )

set(occ-map_url https://github.com/openhumanoids/occ-map.git)
set(occ-map_revision 34ab71fa693216d2c0508f0f2680b9a68994f473)
set(occ-map_depends libbot ${opencv_proj})

set(common_utils_url https://github.com/openhumanoids/common_utils.git)
set(common_utils_revision bf0c9223e02a193a3cfef4034ef82a94219f116a)
set(common_utils_depends Eigen_pod libbot occ-map octomap)

set(frsm_url https://github.com/openhumanoids/frsm.git)
set(frsm_revision 0fb51ac1580a7f6ce3b5f4d7afc302caa976246f)
set(frsm_depends libbot)

set(kinect_url https://github.com/openhumanoids/kinect.git)
set(kinect_revision 1b7c944c08ba4e3a488298d1cdbf6f8485fb015e)
set(kinect_depends libbot)

set(microstrain_url https://github.com/openhumanoids/microstrain.git)
set(microstrain_revision 93f4582491f0cfec1c658ed3fb84ae84a13bc6c2)
set(microstrain_depends common_utils)

set(bullet_url https://github.com/RobotLocomotion/bullet-pod.git)
set(bullet_revision 24b0a184e177c793a1b2f37f55d3544f2a7c33ae)
set(bullet_depends)

set(fovis_url https://github.com/fovis/fovis.git)
#set(fovis_revision 2862e080dee2519932585b46cc301a38d8cad1f4) # this is the newer version, but removes fovis-bot2
set(fovis_revision ee2fe6593ed9e7e5ce2b2f6f1c64b627da119090)
set(fovis_depends libbot)

set(estimate-pose_url https://github.com/openhumanoids/estimate-pose.git)
set(estimate-pose_revision e24a46af00c46116b76f69ff0f20923e60daa519)
set(estimate-pose_depends fovis)

set(vicon_url https://github.com/openhumanoids/vicon.git)
set(vicon_revision 867d2f1192ad37b2c37ad3d7611e40ef27ff349d)
set(vicon_depends libbot)

set(apriltags_url https://github.com/psiorx/apriltags-pod.git)
set(apriltags_revision ed2972f01e00d9b5a4afa2612b018374d889641f)
set(apriltags_depends)

set(spotless_url https://github.com/RobotLocomotion/spotless-pod.git)
set(spotless_revision 91e1e3970e62b8dd8d74bbc4446504b11990f598)
set(spotless_depends)

set(snopt_url ssh://git@github.com/openhumanoids/snopt.git)
set(snopt_revision 95d908275156f2665ef3941f08cb89c767480a6e)
set(snopt_depends)

set(gurobi-private_url ssh://git@github.com/openhumanoids/gurobi-private.git)
set(gurobi-private_revision cfeea24766ea1a11d5fc6eeff193ab520c3e58d2)
set(gurobi-private_depends)
set(gurobi-private_external_args ${download_only_args})

set(gurobi_url https://github.com/RobotLocomotion/gurobi.git)
set(gurobi_revision b95a186b4d988db00ada55bd8efb08c651a83fe7)
if(APPLE)
  set(gurobi_distro_file ${source_prefix}/gurobi-private/gurobi5.6.2_mac64.pkg)
else()
  set(gurobi_distro_file ${source_prefix}/gurobi-private/gurobi5.6.2_linux64.tar.gz)
endif()
set(gurobi_environment_args GUROBI_DISTRO=${gurobi_distro_file})
set(gurobi_depends gurobi-private)

set(atlas-driver_url ssh://git@github.com/openhumanoids/atlas-drivers.git)
set(atlas-driver_revision 04c805efae5913cc2dc9da0734dc4ec585e8d5d4)
set(atlas-driver_depends)
set(atlas-driver_external_args
  ${download_only_args}
  SOURCE_DIR ${source_prefix}/../atlas-collection/atlas
  )

set(flycapture_url ssh://git@github.com/openhumanoids/flycapture-pod.git)
set(flycapture_revision be27acc7effd83b0cb5648742b915eb4a6181f49)
set(flycapture_depends)
set(flycapture_external_args
  ${download_only_args}
  SOURCE_DIR ${source_prefix}/../atlas-collection/flycapture
  )

set(swigmake_url https://github.com/rdeits/swigmake.git)
set(swigmake_revision ab03741a0627e99589ecbc1c088a4db05755e3c2)
set(swigmake_depends )

set(iris_url https://github.com/rdeits/iris-distro.git)
set(iris_revision 7442fbca7a456f5564296902d8e2130a751bf3e3)
set(iris_depends Eigen_pod)
set(iris_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    -DIRIS_WITH_EIGEN:BOOL=OFF
  )

set(pypolyhedron_url https://github.com/rdeits/pypolyhedron.git)
set(pypolyhedron_revision 1f110addf89398f62644830bf69a69930db8c4d0)
set(pypolyhedron_depends)

set(kinematics-utils_url https://github.com/ipab-slmc/kinematics-utils.git)
set(kinematics-utils_revision 123169c091d0160e80de31ddc31efcefbb413fd6)
set(kinematics-utils_depends Eigen_pod)

set(libmultisense_url https://bitbucket.org/crl/libmultisense)
set(libmultisense_hg_tag a57026c)
set(libmultisense_depends ${opencv_proj})
set(libmultisense_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
  )

set(multisense-lcm_url http://github.com/openhumanoids/multisense-lcm.git)
set(multisense-lcm_revision b05368f47219642fdebe06f93fe8193f740ed34f)
set(multisense-lcm_depends libmultisense)

set(libnabo_url https://github.com/ethz-asl/libnabo.git)
set(libnabo_revision 7d6b111de63a5118e11f551e336606233018ee8d)
set(libnabo_depends Eigen_pod)
set(libnabo_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${eigen_args}
    ${python_args}
  )

set(libpointmatcher_url https://github.com/ethz-asl/libpointmatcher.git)
set(libpointmatcher_revision 6c6acbb17199f9a51467dff86875bbc11add5bf2)
set(libpointmatcher_depends Eigen_pod libnabo)
set(libpointmatcher_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${eigen_args}
  )

set(hokuyo_url https://github.com/openhumanoids/hokuyo.git)
set(hokuyo_revision 6fc1b804d80838ae314d162929bb0a25a231ca35)
set(hokuyo_depends libbot)

set(cmake_scripts_url https://github.com/RobotLocomotion/cmake.git)
set(cmake_scripts_revision be0915b23c81d4dce8a6f62e4e74214c7047558a)
set(cmake_scripts_external_args
  ${download_only_args}
  SOURCE_DIR ${source_prefix}/../drake/drake/cmake
  )

set(PythonQt_url https://github.com/commontk/PythonQt.git)
set(PythonQt_revision 00e6c6b2)
set(PythonQt_depends)
set(PythonQt_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${python_args}
    ${qt_args}
    -DPythonQt_Wrap_Qtcore:BOOL=ON
    -DPythonQt_Wrap_Qtgui:BOOL=ON
    -DPythonQt_Wrap_Qtuitools:BOOL=ON
  )

set(ctkPythonConsole_url https://github.com/patmarion/ctkPythonConsole.git)
set(ctkPythonConsole_revision 15988c5)
set(ctkPythonConsole_depends PythonQt)
set(ctkPythonConsole_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${python_args}
    ${qt_args}
  )

set(QtPropertyBrowser_url https://github.com/patmarion/QtPropertyBrowser.git)
set(QtPropertyBrowser_revision baf10af)
set(QtPropertyBrowser_depends)
set(QtPropertyBrowser_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${qt_args}
  )

set(PointCloudLibraryPlugin_url https://github.com/patmarion/PointCloudLibraryPlugin.git)
set(PointCloudLibraryPlugin_revision cb119b0)
set(PointCloudLibraryPlugin_depends ${pcl_proj})
set(PointCloudLibraryPlugin_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${vtk_args}
    -DPCL_REQUIRED_VERSION:STRING=1.7
  )


set(isam_url https://github.com/ipab-slmc/isam.git)
set(isam_revision a6795ce22a8a90cdf87e3d1306af93adbeec1aeb)
set(isam_depends)
set(isam_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${eigen_args}
  )

set(externals
  Eigen_pod
  ${lcm_proj}
  bot_core_lcmtypes
  libbot
  ${opencv_proj}
  flann
  ${pcl_proj}
  octomap
  occ-map
  common_utils
  frsm
  fovis
  estimate-pose
  apriltags
  bullet
  spotless
  swigmake
  iris
  pypolyhedron
  libnabo
  libpointmatcher
  hokuyo
  cmake_scripts
  PythonQt
  ctkPythonConsole
  QtPropertyBrowser
  PointCloudLibraryPlugin
  isam
  )

if(BUILD_PRIVATE_EXTERNALS)

  list(APPEND externals
    atlas-driver
    gurobi-private
    gurobi
    snopt
    flycapture
    )

endif()


if(NOT APPLE)

  # These modules only compile on linux. Some have strict requirements on
  # linux, others could be fixed up to support mac osx builds.
  list(APPEND externals
    libmultisense
    multisense-lcm
    microstrain
    vicon
    kinematics-utils
  )

  list(FIND externals fovis fovis_index)
  list(INSERT externals ${fovis_index} kinect)
  list(APPEND fovis_depends kinect)

endif()


# Checks whether Matlab is installed, else remove dependent packages
find_program(matlab matlab)
if (NOT matlab)
  message(WARNING "Could not find matlab executable - not building spotless")
  list(REMOVE_ITEM externals
    spotless
  )
endif()


macro(add_external proj)

  # depending on which variables are defined, the external project
  # might be mercurial, svn, git, or an archive download.
  if(DEFINED ${proj}_hg_tag)
    set(download_args
      HG_REPOSITORY ${${proj}_url}
      HG_TAG ${${proj}_hg_tag})
  elseif(DEFINED ${proj}_svn_revision)
    set(download_args
    SVN_REPOSITORY ${${proj}_url}
    SVN_REVISION -r ${${proj}_revision})
  elseif(DEFINED ${proj}_download_hash)
    set(download_args
    URL ${${proj}_url}
    URL_MD5 ${${proj}_download_hash})
  else()
    set(download_args
      GIT_REPOSITORY ${${proj}_url}
      GIT_TAG ${${proj}_revision})
  endif()

  # if this variable is not defined then this external will be treated as
  # a standard pod so we'll define the required configure and build commands
  if(NOT DEFINED ${proj}_external_args)
    set(pod_build_args
      CONFIGURE_COMMAND ${empty_command}
      INSTALL_COMMAND ${empty_command}
      BUILD_COMMAND $(MAKE) BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE} ${${proj}_environment_args}
      BUILD_IN_SOURCE 1
    )
    set(${proj}_external_args ${pod_build_args})
    set(${proj}_is_pod TRUE)
  endif()

  # this supports non-standard download locations
  set(source_dir_arg)
  list(FIND ${proj}_external_args SOURCE_DIR res)
  if (res EQUAL -1)
    set(source_dir_arg SOURCE_DIR ${source_prefix}/${proj})
  endif()

  # workaround a cmake issue, we need to support empty strings as list elements
  # so replace the string NONE with empty string here right before arg conversion
  # and then the variable will be quoted in the following call to ExternalProject_Add.
  string(REGEX REPLACE "NONE" "" ${proj}_external_args "${${proj}_external_args}")

  ExternalProject_Add(${proj}
    DEPENDS ${${proj}_depends}
    ${download_args}
    ${source_dir_arg}
    "${${proj}_external_args}"
    )

endmacro()


foreach(external ${externals})
  add_external(${external})
endforeach()
