classdef AffIndexedTrajOptConstraintListener
    properties
        lc
        aggregator
    end
    
    methods
        function obj = AffIndexedTrajOptConstraintListener(channel)
            obj.lc = lcm.lcm.LCM.getSingleton();
            obj.aggregator = lcm.lcm.MessageAggregator();
            obj.lc.subscribe(channel, obj.aggregator);
        end
        
        function X = getNextMessage(obj, t_ms)
            msg_raw = obj.aggregator.getNextMessage(t_ms);
            
            if isempty(msg_raw)
                X = [];
            else
                msg = drc.aff_indexed_traj_opt_constraint_t(msg_raw.data);
                X = AffIndexedTrajOptConstraintListener.decodeAffIndexedTrajOptConstraint(msg);
            end
        end
        
    end
    
    methods(Static)
        function X = decodeAffIndexedTrajOptConstraint(msg)
            X = [];
            
            for j = 1:msg.num_links,
                X(j).time = double(msg.link_aff_index(j).utime)/1000000;
                X(j).aff_type =msg.link_aff_index(j).aff_type;
                X(j).aff_uid =msg.link_aff_index(j).aff_uid;
                X(j).num_ees = msg.link_aff_index(j).num_ees;
                X(j).ee_name = msg.link_aff_index(j).ee_name;
                X(j).dof_name = msg.link_aff_index(j).dof_name;
                X(j).dof_value = msg.link_aff_index(j).dof_value;
                X(j).name = msg.link_name(j);
                X(j).desired_pose = [msg.link_origin_position(j).translation.x;
                    msg.link_origin_position(j).translation.y;
                    msg.link_origin_position(j).translation.z;
                    msg.link_origin_position(j).rotation.w;
                    msg.link_origin_position(j).rotation.x;
                    msg.link_origin_position(j).rotation.y;
                    msg.link_origin_position(j).rotation.z];
                
            end
        end
    end
end
