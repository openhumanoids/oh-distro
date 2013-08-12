FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sandia_hand_msgs/msg"
  "../src/sandia_hand_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/SimpleGraspSrv.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SimpleGraspSrv.lisp"
  "../srv_gen/lisp/SetFingerHome.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SetFingerHome.lisp"
  "../srv_gen/lisp/GetParameters.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_GetParameters.lisp"
  "../srv_gen/lisp/SetJointLimitPolicy.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SetJointLimitPolicy.lisp"
  "../srv_gen/lisp/SetParameters.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SetParameters.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
