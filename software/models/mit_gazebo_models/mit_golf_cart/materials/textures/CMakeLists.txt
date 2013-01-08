set (files 
  texture0.jpg
  texture1.jpg
  texture2.jpg
  texture3.jpg
  texture4.jpg
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/materials/textures/)
add_subdirectory(no_wheels)
