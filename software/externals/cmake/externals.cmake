set(lcm_pod_url https://github.com/RobotLocomotion/lcm-pod.git)
set(lcm_pod_revision c43ae7d)
set(lcm_pod_depends)

set(libbot-drc_url https://svn.csail.mit.edu/drc/trunk/software/externals/libbot-drc)
set(libbot-drc_revision 8334)
set(libbot-drc_depends)

set(Eigen_pod_url https://github.com/RobotLocomotion/eigen-pod.git)
set(Eigen_pod_revision 0f940b6)
set(Eigen_pod_depends)

set(opencv-drc_url https://svn.csail.mit.edu/drc/trunk/software/externals/opencv-drc)
set(opencv-drc_revision 8298)
set(opencv-drc_depends Eigen_pod)

set(pcl_dep_url https://svn.csail.mit.edu/drc/trunk/software/externals/pcl_dep)
set(pcl_dep_revision 8337)
set(pcl_dep_depends)

set(pcl_drc_url https://svn.csail.mit.edu/drc/trunk/software/externals/pcl_drc)
set(pcl_drc_revision 8326)
set(pcl_drc_depends Eigen_pod pcl_dep)

set(pcl_url http://github.com/pointcloudlibrary/pcl.git)
set(pcl_revision pcl-1.7.1)
set(pcl_depends pcl_dep Eigen_pod)
set(pcl_external_args
  CMAKE_CACHE_ARGS
    -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  )

set(octomap-drc_url https://svn.csail.mit.edu/drc/trunk/software/externals/octomap-drc)
set(octomap-drc_revision 8327)
set(octomap-drc_depends)

set(occ-map_url https://svn.csail.mit.edu/rrg_pods/Isam_Slam/occ-map)
set(occ-map_revision 958)
set(occ-map_depends libbot-drc opencv-drc)

set(common_utils_url https://svn.csail.mit.edu/common_utils)
set(common_utils_revision 301)
set(common_utils_depends Eigen_pod libbot-drc occ-map octomap-drc)

set(scanmatch_url https://svn.csail.mit.edu/scanmatch/trunk)
set(scanmatch_revision 49)
set(scanmatch_depends opencv-drc)

set(jpeg-utils_url https://svn.csail.mit.edu/drc/trunk/software/externals/jpeg-utils)
set(jpeg-utils_revision 8298)
set(jpeg-utils_depends)

set(isam_include_url https://svn.csail.mit.edu/drc/trunk/software/externals/isam_include)
set(isam_include_revision 8298)
set(isam_include_depends)

set(visualization_url https://svn.csail.mit.edu/drc/trunk/software/externals/visualization)
set(visualization_revision 8321)
set(visualization_depends isam_include Eigen_pod libbot-drc)

set(velodyne_url https://svn.csail.mit.edu/rrg_pods/drivers/velodyne)
set(velodyne_revision 852)
set(velodyne_depends common_utils)

set(kinect_url https://svn.csail.mit.edu/rrg_pods/drivers/kinect)
set(kinect_revision 964)
set(kinect_depends libbot-drc)

set(microstrain_comm_url https://svn.csail.mit.edu/rrg_pods/drivers/microstrain_comm)
set(microstrain_comm_revision 853)
set(microstrain_comm_depends common_utils)

set(bullet_url https://github.com/RobotLocomotion/bullet-pod.git)
set(bullet_revision fd4a647)
set(bullet_depends)

set(fovis-git_url https://svn.csail.mit.edu/drc/trunk/software/externals/fovis-git)
set(fovis-git_revision 8298)
set(fovis-git_depends libbot-drc Eigen_pod )

set(estimate-pose_url https://svn.csail.mit.edu/rrg_pods/estimate-pose)
set(estimate-pose_revision 827)
set(estimate-pose_depends fovis-git)

set(vicon_url https://svn.csail.mit.edu/rrg_pods/drivers/vicon)
set(vicon_revision 855)
set(vicon_depends libbot-drc)

set(vicon-drc_url https://svn.csail.mit.edu/drc/trunk/software/externals/vicon-drc)
set(vicon-drc_revision 8298)
set(vicon-drc_depends)

set(camunits-wrapper_url https://svn.csail.mit.edu/rrg_pods/camunits-pods/camunits-wrapper)
set(camunits-wrapper_revision 887)
set(camunits-wrapper_depends)

set(camunits-extra-wrapper_url https://svn.csail.mit.edu/rrg_pods/camunits-pods/camunits-extra-wrapper)
set(camunits-extra-wrapper_revision 887)
set(camunits-extra-wrapper_depends camunits-wrapper)

set(apriltags_url https://github.com/psiorx/apriltags-pod.git)
set(apriltags_revision ed2972f01e00d9b5a4afa2612b018374d889641f)
set(apriltags_depends)

set(spotless_url ssh://git@github.com/RobotLocomotion/spotless-pod.git)
set(spotless_revision 464be854a1296d4726cb37d86f24d39742293ab6)
set(spotless_depends)

set(snopt_url ssh://git@github.com/RobotLocomotion/snopt.git)
set(snopt_revision 4ab2fd0e4d57ef906bf2a655093824056e2831ba)
set(snopt_depends)

set(gurobi_url ssh://git@github.com/RobotLocomotion/gurobi.git)
set(gurobi_revision b95a186b4d988db00ada55bd8efb08c651a83fe7)
set(gurobi_environment_args GUROBI_DISTRO=${CMAKE_CURRENT_SOURCE_DIR}/cmake/gurobi5.6.2_linux64.tar.gz)
set(gurobi_depends)

set(iris_url ssh://git@github.com/rdeits/iris-distro.git)
set(iris_revision 1f51e5089cd477227a300d4cc625375f2b26cd17)
set(iris_depends)

set(mosek_url ssh://git@github.com/RobotLocomotion/mosek.git)
set(mosek_revision fa997f27ffc309992909e396fece67086011258f)
set(mosek_depends)

set(flycapture_url https://svn.csail.mit.edu/drc/trunk/software/externals/flycapture)
set(flycapture_revision 8298)
set(flycapture_depends)

set(pypolyhedron_url ssh://git@github.com/rdeits/pypolyhedron.git)
set(pypolyhedron_revision 1f110addf89398f62644830bf69a69930db8c4d0)
set(pypolyhedron_depends)

set(kinematics-utils_url ssh://git@github.com/mitdrc/kinematics-utils.git)
set(kinematics-utils_revision 062e36b056eaa0b5ddc38ed7c738999b9bb5831b)
set(kinematics-utils_depends Eigen_pod)

set(libmultisense_url https://bitbucket.org/crl/libmultisense)
set(libmultisense_hg_tag a57026c)
set(libmultisense_depends opencv-drc)
set(libmultisense_external_args
  CMAKE_CACHE_ARGS
    -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  )

set(multisense-lcm_url http://github.com/mitdrc/multisense-lcm.git)
set(multisense-lcm_revision b05368f47219642fdebe06f93fe8193f740ed34f)
set(multisense-lcm_depends libmultisense)


set(externals
  Eigen_pod
  libbot-drc
  opencv-drc
  pcl_dep
  #pcl_drc
  pcl
  octomap-drc
  libmultisense
  multisense-lcm
  occ-map
  common_utils
  #scanmatch
  jpeg-utils
  isam_include
  visualization
  #velodyne
  kinect
  microstrain_comm
  fovis-git
  estimate-pose
  vicon
  vicon-drc
  #camunits-wrapper
  #camunits-extra-wrapper
  apriltags
  flycapture
  bullet
  spotless
  snopt
  gurobi
  iris
  mosek
  pypolyhedron
  kinematics-utils
  )


set(svn_credentials)
if(DRC_SVN_PASSWORD)
  set(svn_credentials SVN_USERNAME drc SVN_PASSWORD ${DRC_SVN_PASSWORD})
endif()

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
    ${svn_credentials}
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
