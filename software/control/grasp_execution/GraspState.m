classdef GraspState < LCMCoordinateFrameWCoder & Singleton
    
    methods
        function obj = GraspState(r_left,r_right)
            
            nq_l = r_left.getNumStates/2;
            l_jointNames = r_left.getStateFrame.coordinates(7:nq_l);
            nq_r = r_right.getNumStates/2;
            r_jointNames = r_right.getStateFrame.coordinates(7:nq_r);

            % Temporary fix as URDFs for sticky hands are not up to date.
            % l_jointNames = regexprep(l_jointNames,'left_','l_');
            % r_jointNames = regexprep(r_jointNames,'right_','r_');
            
          
            nx=18+nq_l-6+nq_r-6;
          
            %public GraspStateCoder(String robot_name, String[] l_joint_name, String[] r_joint_name)
            coder = GraspStateCoder('atlas',l_jointNames,r_jointNames);            
            obj = obj@LCMCoordinateFrameWCoder('grasp_state',nx,'x',JLCMCoder(coder));
            coordNames = cell(nx,1);
            
            % fdata.val = [unique_id;grasp_type;
            %             l_hand_pose.translation;l_hand_pose.rotation;
            %             r_hand_pose.translation;r_hand_pose.rotation;
            %             num_l_joints;num_r_joints;
            %             l_joint_position[1, ...,num_l_joints]
            %             r_joint_position[1, ...,num_r_joints]];
            coordNames{1} = 'unique_id';
            coordNames{2} = 'grasp_type';
            coordNames{3} = 'l_hand_pose_x';
            coordNames{4} = 'l_hand_pose_y';
            coordNames{5} = 'l_hand_pose_z';
            coordNames{6} = 'l_hand_pose_quat_x';
            coordNames{7} = 'l_hand_pose_quat_y';
            coordNames{8} = 'l_hand_pose_quat_z';
            coordNames{9} = 'l_hand_pose_quat_w';
            coordNames{10} = 'r_hand_pose_x';
            coordNames{11} = 'r_hand_pose_y';
            coordNames{12} = 'r_hand_pose_z';
            coordNames{13} = 'r_hand_pose_quat_x';
            coordNames{14} = 'r_hand_pose_quat_y';
            coordNames{15} = 'r_hand_pose_quat_z';
            coordNames{16} = 'r_hand_pose_quat_w';
            coordNames{17} = 'num_l_joints';
            coordNames{18} = 'num_r_joints';
            offset = 19;
            for i = offset:(offset+nq_l-6-1)
                coordNames{i} = l_jointNames{i-offset+1};
            end
            offset = offset+nq_l-6;
            for i = (offset):(offset+nq_r-6-1)
                coordNames{i} = r_jointNames{i-offset+1};
            end
            obj.setCoordinateNames(coordNames); % is this required?
            obj.setDefaultChannel('COMMITTED_GRASP_SEED');
        end
    end
end
