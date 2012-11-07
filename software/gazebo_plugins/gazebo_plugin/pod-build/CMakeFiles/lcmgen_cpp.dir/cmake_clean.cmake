FILE(REMOVE_RECURSE
  "../lcmtypes/c"
  "../lcmtypes/cpp"
  "../lcmtypes/java"
  "../lcmtypes/python"
  "CMakeFiles/lcmgen_cpp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/lcmgen_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
