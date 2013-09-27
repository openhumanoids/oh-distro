#execute_process(COMMAND pwd 
#    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/floatingbase_urdfs/xacro RESULT_VARIABLE rv)
# message("rv='${rv}'") 

execute_process(COMMAND rosrun xacro xacro.py atlas.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../model_minimal_contact.urdf)

execute_process(COMMAND rosrun xacro xacro.py atlas_full_contact.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../model.urdf)

execute_process(COMMAND rosrun xacro xacro.py atlas_fixedjoint_hands.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../model_minimal_contact_fixedjoint_hands.urdf)
    
execute_process(COMMAND rosrun xacro xacro.py atlas_point_sandia_hands.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../model_minimal_contact_point_hands.urdf)    

execute_process(COMMAND rosrun xacro xacro.py atlas_point_stumps.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../model_minimal_contact_point_stumps.urdf)    
   
message(STATUS "Generated \n model_minimal_contact.urdf \n model.urdf \n model_minimal_contact_fixedjoint_hands.urdf \n model_minimal_contact_point_hands.urdf \n model_minimal_contact_point_stumps.urdf")
