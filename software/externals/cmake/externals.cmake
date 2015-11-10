set(lcm_pod_url https://github.com/RobotLocomotion/lcm-pod.git)
set(lcm_pod_revision c43ae7d)
set(lcm_pod_depends)

set(libbot_url https://github.com/openhumanoids/libbot.git)
set(libbot_revision eb61f9bed5df654ffb0ce47ce44ca3d379a6e6e8)
set(libbot_depends)

set(Eigen_pod_url https://github.com/RobotLocomotion/eigen-pod.git)
set(Eigen_pod_revision 3ec6bbfcb41a9ea74921720d62f86863fdf96217)
set(Eigen_pod_depends)

set(opencv-pod_url https://github.com/openhumanoids/opencv-pod.git)
set(opencv-pod_revision 7f4c81676f887ee64da446f1ba61a3e5ed80f879)
set(opencv-pod_depends Eigen_pod)

set(flann-pod_url https://github.com/openhumanoids/flann-pod.git)
set(flann-pod_revision 9619e03a1d1f8b4b27d370810b9b2ccfe00adf58)
set(flann-pod_depends)

set(pcl_url http://github.com/pointcloudlibrary/pcl.git)
set(pcl_revision pcl-1.7.1)
set(pcl_depends flann-pod Eigen_pod)
set(pcl_external_args
  CMAKE_CACHE_ARGS
    -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  )

set(octomap-pod_url https://github.com/openhumanoids/octomap-pod.git)
set(octomap-pod_revision 590b430dfe9f949118fdd618b620d82d84eb0162)
set(octomap-pod_depends)

set(occ-map_url https://github.com/openhumanoids/occ-map.git)
set(occ-map_revision 34ab71fa693216d2c0508f0f2680b9a68994f473)
set(occ-map_depends libbot opencv-pod)

set(common_utils_url https://github.com/openhumanoids/common_utils.git)
set(common_utils_revision 7fc252e3b1baf8ec6a19ee812ae114e8ee460dbd)
set(common_utils_depends Eigen_pod libbot occ-map octomap-pod)

set(frsm_url ssh://git@github.com/openhumanoids/frsm.git)
set(frsm_revision bbf1d83023ae35ecf595c93e46b8601206be9e9b)
set(frsm_depends libbot)

set(kinect_url https://github.com/openhumanoids/kinect.git)
set(kinect_revision 1b7c944c08ba4e3a488298d1cdbf6f8485fb015e)
set(kinect_depends libbot)

set(microstrain_url https://github.com/openhumanoids/microstrain.git)
set(microstrain_revision 93f4582491f0cfec1c658ed3fb84ae84a13bc6c2)
set(microstrain_depends common_utils)

set(bullet_url https://github.com/RobotLocomotion/bullet-pod.git)
set(bullet_revision 4319ffd7e9251066d93064f5a8dab12e33dbe5e2)
set(bullet_depends)

set(fovis_url ssh://git@github.com/fovis/fovis.git)
set(fovis_revision ee2fe6593ed9e7e5ce2b2f6f1c64b627da119090)
set(fovis_depends libbot kinect)

set(estimate-pose_url https://github.com/openhumanoids/estimate-pose.git)
set(estimate-pose_revision e24a46af00c46116b76f69ff0f20923e60daa519)
set(estimate-pose_depends fovis)

set(vicon_url https://github.com/openhumanoids/vicon.git)
set(vicon_revision 867d2f1192ad37b2c37ad3d7611e40ef27ff349d)
set(vicon_depends libbot)

set(apriltags_url https://github.com/psiorx/apriltags-pod.git)
set(apriltags_revision ed2972f01e00d9b5a4afa2612b018374d889641f)
set(apriltags_depends)

set(spotless_url ssh://git@github.com/RobotLocomotion/spotless-pod.git)
set(spotless_revision 91e1e3970e62b8dd8d74bbc4446504b11990f598)
set(spotless_depends)

set(snopt_url ssh://git@github.com/openhumanoids/snopt.git)
set(snopt_revision 95d908275156f2665ef3941f08cb89c767480a6e)
set(snopt_depends)

set(gurobi_url ssh://git@github.com/RobotLocomotion/gurobi.git)
set(gurobi_revision b95a186b4d988db00ada55bd8efb08c651a83fe7)
set(gurobi_environment_args GUROBI_DISTRO=${CMAKE_CURRENT_SOURCE_DIR}/cmake/gurobi5.6.2_linux64.tar.gz)
set(gurobi_depends)

set(iris_url ssh://git@github.com/openhumanoids/iris-distro.git)
set(iris_revision b278a85ba4b3b3da3af41feb39c13e4150d12d98)
set(iris_depends Eigen_pod)
set(iris_external_args
  CMAKE_CACHE_ARGS
    -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DIRIS_WITH_EIGEN:BOOL=OFF
  )

# not installed anymore because iris-distro installs mosek
set(mosek_url ssh://git@github.com/RobotLocomotion/mosek.git)
set(mosek_revision bb7000e)
set(mosek_depends)

set(pypolyhedron_url ssh://git@github.com/rdeits/pypolyhedron.git)
set(pypolyhedron_revision 1f110addf89398f62644830bf69a69930db8c4d0)
set(pypolyhedron_depends)

set(kinematics-utils_url ssh://git@github.com/ipab-slmc/kinematics-utils.git)
set(kinematics-utils_revision 062e36b056eaa0b5ddc38ed7c738999b9bb5831b)
set(kinematics-utils_depends Eigen_pod)

set(libmultisense_url https://bitbucket.org/crl/libmultisense)
set(libmultisense_hg_tag a57026c)
set(libmultisense_depends opencv-pod)
set(libmultisense_external_args
  CMAKE_CACHE_ARGS
    -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  )

set(multisense-lcm_url http://github.com/openhumanoids/multisense-lcm.git)
set(multisense-lcm_revision b05368f47219642fdebe06f93fe8193f740ed34f)
set(multisense-lcm_depends libmultisense)

set(libnabo_url https://github.com/ethz-asl/libnabo.git)
set(libnabo_revision 7d6b111de63a5118e11f551e336606233018ee8d)
set(libnabo_depends Eigen_pod)
set(libnabo_external_args
  CMAKE_CACHE_ARGS
    -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo
  )

set(libpointmatcher_url https://github.com/ethz-asl/libpointmatcher.git)
set(libpointmatcher_revision 6c6acbb17199f9a51467dff86875bbc11add5bf2)
set(libpointmatcher_depends Eigen_pod libnabo)
set(libpointmatcher_external_args
  CMAKE_CACHE_ARGS
    -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo
  )

set(hokuyo_url ssh://git@github.com/openhumanoids/hokuyo.git)
set(hokuyo_revision 6fc1b804d80838ae314d162929bb0a25a231ca35)
set(hokuyo_depends libbot)

set(externals
  Eigen_pod
  libbot
  opencv-pod
  flann-pod
  pcl
  octomap-pod
  libmultisense
  multisense-lcm
  occ-map
  common_utils
  frsm
  kinect
  microstrain
  fovis
  estimate-pose
  vicon
  apriltags
  bullet
  spotless
  snopt
  gurobi
  iris
  pypolyhedron
  kinematics-utils
  libnabo
  libpointmatcher
  hokuyo
  )



macro(add_external proj)
  if (DEFINED ${proj}_hg_tag)
    add_mercurial_external(${proj})
  elseif (${${proj}_url} MATCHES "\\.git$")
    add_git_external(${proj})
  else()
    add_svn_external(${proj})
  endif()
endmacro()

macro(add_svn_external proj)
  ExternalProject_Add(${proj}
    SVN_REPOSITORY ${${proj}_url}
    SVN_REVISION -r ${${proj}_revision}
    DEPENDS ${${proj}_depends}
    CONFIGURE_COMMAND ""
    INSTALL_COMMAND ""
    BUILD_COMMAND $(MAKE) BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE}  ${${proj}_environment_args}
    BUILD_IN_SOURCE 1
    SOURCE_DIR ${DRCExternals_SOURCE_DIR}/${proj}
    )
endmacro()

macro(add_git_external proj)

  if (DEFINED ${proj}_external_args)
    ExternalProject_Add(${proj}
      GIT_REPOSITORY ${${proj}_url}
      GIT_TAG ${${proj}_revision}
      DEPENDS ${${proj}_depends}
      SOURCE_DIR ${DRCExternals_SOURCE_DIR}/${proj}
      ${${proj}_external_args}
      )
  else()
    ExternalProject_Add(${proj}
      GIT_REPOSITORY ${${proj}_url}
      GIT_TAG ${${proj}_revision}
      DEPENDS ${${proj}_depends}
      CONFIGURE_COMMAND ""
      INSTALL_COMMAND ""
      BUILD_COMMAND $(MAKE) BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE} ${${proj}_environment_args}
      BUILD_IN_SOURCE 1
      SOURCE_DIR ${DRCExternals_SOURCE_DIR}/${proj}
      )
    endif()
endmacro()


macro(add_mercurial_external proj)
    ExternalProject_Add(${proj}
      HG_REPOSITORY ${${proj}_url}
      HG_TAG ${${proj}_hg_tag}
      DEPENDS ${${proj}_depends}
      SOURCE_DIR ${DRCExternals_SOURCE_DIR}/${proj}
      ${${proj}_external_args}
      )
endmacro()


foreach(external ${externals})
  add_external(${external})
endforeach()


# Eigen will install eigen3.pc to build/share instead of build/lib
# unless this directory is created before Eigen configures.
ExternalProject_Add_Step(Eigen_pod make_pkgconfig_dir
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig
  DEPENDERS configure)


# This can be removed after CRL updates their install rule for libmultisense
ExternalProject_Add_Step(libmultisense copy_include_dir
  COMMAND cp -r ${DRCExternals_SOURCE_DIR}/libmultisense/source/LibMultiSense/details ${CMAKE_INSTALL_PREFIX}/include/MultiSense/
  DEPENDEES install)

# Install Drake CMake scripts repository
set(cmake_GIT_TAG 2c4ee11aa719ad548df7eaf3c7047a98c6e3e01c)
ExternalProject_Add(drake-cmake
  GIT_REPOSITORY https://github.com/RobotLocomotion/cmake.git
  GIT_TAG ${cmake_GIT_TAG}
  SOURCE_DIR ${DRCExternals_SOURCE_DIR}/../drake/drake/cmake
	CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  )
