#execute_process(COMMAND pwd 
#    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/floatingbase_urdfs/xacro RESULT_VARIABLE rv)
# message("rv='${rv}'") 

execute_process(COMMAND rosrun xacro xacro.py sandia_hands_w_floating_bases.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/floatingbase_urdfs/xacro OUTPUT_FILE ../sandia_hands_floating.urdf)

execute_process(COMMAND rosrun xacro xacro.py sandia_hand_drake_right_w_floating_base.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/floatingbase_urdfs/xacro OUTPUT_FILE ../sandia_hand_drake_right_floating.urdf)
    
execute_process(COMMAND rosrun xacro xacro.py sandia_hand_drake_left_w_floating_base.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/floatingbase_urdfs/xacro OUTPUT_FILE ../sandia_hand_drake_left_floating.urdf)
   
message(STATUS "Generated \n sandia_hands_floating.urdf,\n sandia_hand_drake_left_floating.urdf\n and\n sandia_hand_drake_right_floating.urdf")
