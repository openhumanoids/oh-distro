FILE(REMOVE_RECURSE
  "CMakeFiles/my_plugin.dir/my_plugin.cc.o"
  "../../lib/libmy_plugin.pdb"
  "../../lib/libmy_plugin.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/my_plugin.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
