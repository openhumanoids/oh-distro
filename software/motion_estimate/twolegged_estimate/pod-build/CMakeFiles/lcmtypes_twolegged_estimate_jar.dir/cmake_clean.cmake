FILE(REMOVE_RECURSE
  "../lcmtypes/c"
  "../lcmtypes/cpp"
  "../lcmtypes/java"
  "../lcmtypes/python"
  "CMakeFiles/lcmtypes_twolegged_estimate_jar"
  "lcmtypes_twolegged_estimate.jar"
  "../lcmtypes/java/exlcm/example_t.class"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/lcmtypes_twolegged_estimate_jar.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
