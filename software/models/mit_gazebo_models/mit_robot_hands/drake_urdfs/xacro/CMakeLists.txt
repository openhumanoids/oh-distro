#execute_process(COMMAND pwd 
#    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/floatingbase_urdfs/xacro RESULT_VARIABLE rv)
# message("rv='${rv}'") 

execute_process(COMMAND rosrun xacro xacro.py sandia_hand_drake_left.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/drake_urdfs/xacro OUTPUT_FILE ../sandia_hand_drake_left.urdf)
    
execute_process(COMMAND rosrun xacro xacro.py sandia_hand_drake_right.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/drake_urdfs/xacro OUTPUT_FILE ../sandia_hand_drake_right.urdf)
   
message(STATUS "Generated \n sandia_hand_drake_left.urdf\n and \n sandia_hand_drake_right.urdf")
