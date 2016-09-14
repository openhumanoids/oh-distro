#left leg
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_a_009.xml -j pelvis/left_leg_j1
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_b_016.xml -j left_leg/j2
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_c_008.xml -j left_leg/j3
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_d_008.xml -j left_leg/j4
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_e_016.xml -j left_leg/ankle/left_actuator
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_e_015.xml -j left_leg/ankle/right_actuator
# right leg
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_a_007.xml -j pelvis/right_leg_j1
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_b_015.xml -j right_leg/j2
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_c_009.xml -j right_leg/j3
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_d_009.xml -j right_leg/j4
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_e_021.xml -j right_leg/ankle/left_actuator
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_e_022.xml -j right_leg/ankle/right_actuator
# left arm
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_a_002.xml -j trunk/left_arm_j1
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_b_005.xml -j left_arm/j2
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_f_007.xml -j left_arm/j3
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_f_008.xml -j left_arm/j4
# right arm
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_a_010.xml -j trunk/right_arm_j1
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_b_008.xml -j right_arm/j2
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_f_009.xml -j right_arm/j3
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_f_010.xml -j right_arm/j4
# spine
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_a_008.xml -j pelvis/waist
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_e_012.xml -j trunk/left_actuator
cfgLoader -f /home/val/val_indigo/install/share/val_description/instance/coefficients/actuators/v_e_011.xml -j trunk/right_actuator
