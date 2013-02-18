classdef SandiaJointCommand < LCMCoordinateFrameWCoder
%classdef SandiaJointCommand < LCMCoordinateFrameWCoder & Singleton
    methods
        function obj = SandiaJointCommand(r,side)
            nq = r.getNumStates/2;
            joint_names = r.getStateFrame.coordinates(1:nq);

            floating =true;
            if(floating)
              Kp = [80 80 80   40 40 5    15 10 10    15 10 10    15 10 10    15 10 10   ]';
              Kd = [34 34 34   14 14 2.5  1.5 0.9 0.6 1.5 0.9 0.6 1.5 0.9 0.6 1.5 0.9 0.6]';
            else
              Kp = [15 10 10    15 10 10    15 10 10    15 10 10   ]';
              Kd = [1.5 0.9 0.6 1.5 0.9 0.6 1.5 0.9 0.6 1.5 0.9 0.6]';          
            end
            
            coder = SandiaJointCommandCoder('atlas',floating,side, joint_names,Kp,Kd); 
            obj = obj@LCMCoordinateFrameWCoder('sandia_qd', nq,'q',JLCMCoder(coder));
            obj.setCoordinateNames(joint_names);
            if(strcmp(side,'left'))
              obj.setDefaultChannel('L_HAND_JOINT_COMMANDS');
            elseif(strcmp(side,'right'))
              obj.setDefaultChannel('R_HAND_JOINT_COMMANDS');
            end
        end
    end
end
            
