classdef SandiaPositionRef < LCMCoordinateFrameWCoder & Singleton
    methods
        function obj = SandiaPositionRef(r)
            nq = r.getNumStates/2;
            joint_names = r.getStateFrame.coordinates(1:nq);
%             joint_names = regexprep(joint_names,'_motor',''); % remove motor suffix 
            coder = JointAnglesCoder('sanida_hand', joint_names);
            obj = obj@LCMCoordinateFrameWCoder('sandia_qd', nq,'q',JLCMCoder(coder));
            obj.setCoordinateNames(joint_names);
            obj.setDefaultChannel('JOINT_POSITION_CMDS');
        end
    end
end
            