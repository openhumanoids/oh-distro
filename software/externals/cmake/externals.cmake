
set(libbot-drc_url https://svn.csail.mit.edu/drc/trunk/software/externals/libbot-drc)
set(libbot-drc_revision 8325)
set(libbot-drc_depends)

set(Eigen_pod_url https://svn.csail.mit.edu/Eigen_pod)
set(Eigen_pod_revision 13)
set(Eigen_pod_depends)

set(opencv-drc_url https://svn.csail.mit.edu/drc/trunk/software/externals/opencv-drc)
set(opencv-drc_revision 8298)
set(opencv-drc_depends Eigen_pod)

set(pcl_dep_url https://svn.csail.mit.edu/drc/trunk/software/externals/pcl_dep)
set(pcl_dep_revision 8298)
set(pcl_dep_depends)

set(pcl_drc_url https://svn.csail.mit.edu/drc/trunk/software/externals/pcl_drc)
set(pcl_drc_revision 8298)
set(pcl_drc_depends Eigen_pod pcl_dep)

set(octomap-drc_url https://svn.csail.mit.edu/drc/trunk/software/externals/octomap-drc)
set(octomap-drc_revision 8298)
set(octomap-drc_depends)

set(occ-map_url https://svn.csail.mit.edu/rrg_pods/Isam_Slam/occ-map)
set(occ-map_revision 831)
set(occ-map_depends libbot-drc opencv-drc)

set(common_utils_url https://svn.csail.mit.edu/common_utils)
set(common_utils_revision 287)
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
set(kinect_revision 827)
set(kinect_depends libbot-drc)

set(microstrain_comm_url https://svn.csail.mit.edu/rrg_pods/drivers/microstrain_comm)
set(microstrain_comm_revision 853)
set(microstrain_comm_depends common_utils)

set(bullet_url https://github.com/RobotLocomotion/bullet-pod.git)
set(bullet_revision 029b5258ae08932d2a7edbf1d8de72d316042b8b)
set(bullet_depends)

set(fovis-git_url https://svn.csail.mit.edu/drc/trunk/software/externals/fovis-git)
set(fovis-git_revision 8298)
set(fovis-git_depends libbot-drc Eigen_pod kinect)

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

set(apriltags_url https://svn.csail.mit.edu/apriltags)
set(apriltags_revision 24)
set(apriltags_depends opencv-drc)

set(spotless_url https://svn.csail.mit.edu/locomotion/pods/spotless)
set(spotless_revision 8053)
set(spotless_depends)

set(snopt_url https://svn.csail.mit.edu/locomotion/pods/snopt7.2-12)
set(snopt_revision 7782)
set(snopt_depends)

set(gurobi_url https://svn.csail.mit.edu/locomotion/pods/gurobi)
set(gurobi_revision 8053)
set(gurobi_depends)

set(flycapture_url https://svn.csail.mit.edu/drc/trunk/software/externals/flycapture)
set(flycapture_revision 8298)
set(flycapture_depends)

set(externals
  libbot-drc
  Eigen_pod
  opencv-drc
  pcl_dep
  pcl_drc
  octomap-drc
  occ-map
  common_utils
  scanmatch
  jpeg-utils
  isam_include
  visualization
  velodyne
  kinect
  microstrain_comm
  fovis-git
  estimate-pose
  vicon
  vicon-drc
  camunits-wrapper
  camunits-extra-wrapper
  apriltags
  spotless
  snopt
  gurobi
  flycapture
  )

set(git-externals
  bullet
  )


set(svn_credentials)
if(DRC_SVN_PASSWORD)
  set(svn_credentials SVN_USERNAME drc SVN_PASSWORD ${DRC_SVN_PASSWORD})
endif()

macro(add_svn_external proj)
  ExternalProject_Add(${proj}
    SVN_REPOSITORY ${${proj}_url}
    SVN_REVISION -r ${${proj}_revision}
    ${svn_credentials}
    DEPENDS ${${proj}_depends}
    CONFIGURE_COMMAND ""
    INSTALL_COMMAND ""
    BUILD_COMMAND $(MAKE) BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE}
    BUILD_IN_SOURCE 1
    SOURCE_DIR ${DRCExternals_SOURCE_DIR}/${proj}
    )
endmacro()

macro(add_git_external proj)
  ExternalProject_Add(${proj}
    GIT_REPOSITORY ${${proj}_url}
    GIT_TAG ${${proj}_revision}
    DEPENDS ${${proj}_depends}
    CONFIGURE_COMMAND ""
    INSTALL_COMMAND ""
    BUILD_COMMAND $(MAKE) BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE}
    BUILD_IN_SOURCE 1
    SOURCE_DIR ${DRCExternals_SOURCE_DIR}/${proj}
    )
endmacro()

foreach(external ${externals})
  add_svn_external(${external})
endforeach()

foreach(git-external ${git-externals})
  add_git_external(${git-external})
endforeach()


# Eigen will install eigen3.pc to build/share instead of build/lib
# unless this directory is created before Eigen configures.
ExternalProject_Add_Step(Eigen_pod make_pkgconfig_dir
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig
  DEPENDERS configure)
