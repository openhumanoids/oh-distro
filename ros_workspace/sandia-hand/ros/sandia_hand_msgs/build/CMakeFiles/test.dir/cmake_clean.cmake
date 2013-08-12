FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sandia_hand_msgs/msg"
  "../src/sandia_hand_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/test"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/test.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
