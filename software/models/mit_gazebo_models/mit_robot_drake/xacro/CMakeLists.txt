#execute_process(COMMAND pwd 
#    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/floatingbase_urdfs/xacro RESULT_VARIABLE rv)
# message("rv='${rv}'") 

execute_process(COMMAND rosrun xacro xacro.py atlas.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../model_minimal_contact.urdf)
    
execute_process(COMMAND rosrun xacro xacro.py atlas_sandia_hands.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../model_minimal_contact_with_hands.urdf)
   
message(STATUS "Generated \n model_minimal_contact.urdf \n model_minimal_contact_with_hands.urdf")
