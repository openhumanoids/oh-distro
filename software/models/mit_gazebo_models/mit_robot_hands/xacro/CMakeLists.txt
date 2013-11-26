#execute_process(COMMAND pwd 
#    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/floatingbase_urdfs/xacro RESULT_VARIABLE rv)
# message("rv='${rv}'") 

execute_process(COMMAND rosrun xacro xacro.py sandia_hand_left.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../sandia_hand_left.urdf)
    
execute_process(COMMAND rosrun xacro xacro.py sandia_hand_right.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../sandia_hand_right.urdf)
    
execute_process(COMMAND rosrun xacro xacro.py irobot_hand_left.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../irobot_hand_left.urdf)
    
execute_process(COMMAND rosrun xacro xacro.py irobot_hand_right.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../irobot_hand_right.urdf)
    
execute_process(COMMAND rosrun xacro xacro.py robotiq_hand_left.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../robotiq_hand_left.urdf)
    
execute_process(COMMAND rosrun xacro xacro.py robotiq_hand_right.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../robotiq_hand_right.urdf)    
    
execute_process(COMMAND rosrun xacro xacro.py inert_hand_left.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../inert_hand_left.urdf)
    
execute_process(COMMAND rosrun xacro xacro.py inert_hand_right.urdf.xacro
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/xacro OUTPUT_FILE ../inert_hand_right.urdf)      
   
message(STATUS "Generated sandia/irobot/robotiq/inert hand urdfs for left and right sticky hands\n ")
