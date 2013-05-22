function publishDrivingActuationStatus(state_machine, driving_aff_status_pub)

driving_dof_names = {'steering_joint','gas_joint','brake_joint'};
status_channels = {'DRIVING_STEERING_ACTUATION_STATUS','DRIVING_GAS_ACTUATION_STATUS','DRIVING_BRAKE_ACTUATION_STATUS'};
if (~state_machine.manip_map_received)
    for didx = 1:length(driving_dof_names)
        dof_name = driving_dof_names(didx);
        aff_type = 'unknown';
        aff_uid = -1;
        ee_name = 'unknown';
        have_manip_map = 0;
        dof_value_0 = -1;
        dof_value_1 = -1;
        
        driving_aff_status_pub.publish(status_channels(didx),aff_type, aff_uid, dof_name, ee_name,  have_manip_map, dof_value_0, dof_value_1);
    end;
    return;
end;

dof_names = cellstr(char(state_machine.affinds(1).dof_name));
ee_names = cellstr(char(state_machine.affinds(1).ee_name));
aff_types = cellstr(char(state_machine.affinds(1).aff_type));
for didx = 1:length(driving_dof_names)
    dof_name = driving_dof_names(didx);
    
    
    idx = find(strcmp(char(driving_dof_names(didx)), dof_names));
    if (~isempty(idx))
        aff_type = aff_types(1);
        aff_uid = state_machine.affinds(1).aff_uid;
        ee_name = ee_names(idx);
        
        if (strcmp(char(ee_name),'l_foot'))
            if (state_machine.l_foot_chain.active)
                have_manip_map = 1;
                dof_value_0 = state_machine.l_foot_chain.dof_values(1);
                dof_value_1 = state_machine.l_foot_chain.dof_values(end);
            else
                have_manip_map = 0;
                dof_value_0 = -1;
                dof_value_1 = -1;
            end;
        elseif(strcmp(char(ee_name),'r_foot'))
            if (state_machine.r_foot_chain.active)
                have_manip_map = 1;
                dof_value_0 = state_machine.r_foot_chain.dof_values(1);
                dof_value_1 = state_machine.r_foot_chain.dof_values(end);
            else
                have_manip_map = 0;
                dof_value_0 = -1;
                dof_value_1 = -1;
            end;
        elseif(strcmp(char(ee_name),'l_hand'))
            if (state_machine.l_hand_chain.active)
                have_manip_map = 1;
                dof_value_0 = state_machine.l_hand_chain.dof_values(1);
                dof_value_1 = state_machine.l_hand_chain.dof_values(end);
            else
                have_manip_map = 0;
                dof_value_0 = -1;
                dof_value_1 = -1;
            end;
        elseif(strcmp(char(ee_name),'r_hand'))
            if (state_machine.r_hand_chain.active)
                have_manip_map = 1;
                dof_value_0 = state_machine.r_hand_chain.dof_values(1);
                dof_value_1 = state_machine.r_hand_chain.dof_values(end);
            else
                have_manip_map = 0;
                dof_value_0 = -1;
                dof_value_1 = -1;
            end;
        else
            have_manip_map = 0;
            dof_value_0 = -1;
            dof_value_1 = -1;
        end;
        
    else
        aff_type = 'unknown';
        aff_uid = -1;
        ee_name = 'unknown';
        have_manip_map = 0;
        dof_value_0 = -1;
        dof_value_1 = -1;
    end;
    
    driving_aff_status_pub.publish(status_channels(didx),aff_type, aff_uid, dof_name, ee_name,  have_manip_map, dof_value_0, dof_value_1)
end;