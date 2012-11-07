FILE(REMOVE_RECURSE
  "../lcmtypes/c"
  "../lcmtypes/cpp"
  "../lcmtypes/java"
  "../lcmtypes/python"
  "CMakeFiles/lcmtypes_gazebo_plugin_jar"
  "lcmtypes_gazebo_plugin.jar"
  "../lcmtypes/java/exlcm/example_t.class"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/lcmtypes_gazebo_plugin_jar.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
