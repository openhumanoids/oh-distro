FILE(REMOVE_RECURSE
  "../lcmtypes/c"
  "../lcmtypes/cpp"
  "../lcmtypes/java"
  "../lcmtypes/python"
  "CMakeFiles/test_driver.dir/src/test_driver.cpp.o"
  "bin/test_driver.pdb"
  "bin/test_driver"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/test_driver.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
