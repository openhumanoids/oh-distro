classdef GraspEEGoalCmd < LCMCoordinateFrameWCoder & Singleton
    
    methods
        function obj = GraspEEGoalCmd(r)
            numIn = r.getNumInputs();
            jointNames = r.getInputFrame.coordinates(7:numIn); % Don't care about the base coordinates which are x, y, z, roll, pitch, yaw of the base
            jointNames = regexprep(jointNames,'_motor',''); % remove motor suffix 
            dim = numIn+1;
            coder = GraspGoalCoder('atlas','r_hand', jointNames);
            obj = obj@LCMCoordinateFrameWCoder('grasp_ee_goal_cmd',dim,'i',JLCMCoder(coder));
            coordNames = cell(dim,1);
            coordNames{1} = 'r_hand_x';
            coordNames{2} = 'r_hand_y';
            coordNames{3} = 'r_hand_z';
            coordNames{4} = 'r_hand_quat_x';
            coordNames{5} = 'r_hand_quat_y';
            coordNames{6} = 'r_hand_quat_z';
            coordNames{7} = 'r_hand_quat_w';
            for i = 8:dim
                coordNames{i} = jointNames{i-7};
            end
            obj.setCoordinateNames(coordNames);
            obj.setDefaultChannel('R_HAND_GOAL') % Need to ask Sisir about which channel he is using
        end
    end
end