classdef DrivingAffordanceStatusPublisher
    properties
        lc;
        floating;
    end
    
    methods
        function obj = DrivingAffordanceStatusPublisher(floating)
            %obj.channel = channel;
            obj.lc = lcm.lcm.LCM.getSingleton();
            if nargin == 1
                obj.floating = floating;
            else
                obj.floating = true;
            end
            
        end
        
        function publish(obj, channel, aff_type, aff_uid, dof_name, ee_name,  have_manip_map, dof_value_0, dof_value_1)
            obj.lc.publish(channel, DrivingAffordanceStatusPublisher.encodeDrivingAffordanceStatus(aff_type, aff_uid, dof_name, ee_name,  have_manip_map, dof_value_0, dof_value_1));
        end
        
    end % end methods
    
    methods(Static)
        function msg = encodeDrivingAffordanceStatus(aff_type, aff_uid, dof_name, ee_name,  have_manip_map, dof_value_0, dof_value_1)
            if nargin < 2
                t = now() * 24 * 60 * 60;
            end
            
            t = now() * 24 * 60 * 60;
            msg = drc.driving_affordance_status_t();
            msg.utime = t * 1000000;
            msg.aff_type = aff_type;
            msg.aff_uid = aff_uid;
            msg.dof_name = dof_name;
            msg.ee_name = ee_name;
            msg.have_manip_map = have_manip_map;
            msg.dof_value_0 = dof_value_0;
            msg.dof_value_1 = dof_value_1;
        end;
    end % end methods
end

