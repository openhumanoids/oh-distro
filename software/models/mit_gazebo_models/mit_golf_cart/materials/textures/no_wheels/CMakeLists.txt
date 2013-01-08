set (files 
  __auto_4.jpg
  __auto_2.jpg
  __auto_1.jpg
  __auto_11.jpg
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/materials/textures/no_wheels/)
