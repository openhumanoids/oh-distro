classdef SandiaState < LCMCoordinateFrameWCoder & Singleton
    % Simple coordinate frame for the sandia hand
    methods
        function obj = SandiaState(r)
        typecheck(r,'TimeSteppingRigidBodyManipulator');
        nx=r.getNumStates();
        joint_names = r.getStateFrame.coordinates(1:nx/2);
        coder = RobotStateCoder('sandia_hand',joint_names);
        obj = obj@LCMCoordinateFrameWCoder('SandiaState',nx,'x',JLCMCoder(coder));
        obj.setCoordinateNames(r.getStateFrame.coordinates);
        obj.setDefaultChannel('EST_ROBOT_STATE');
        end
    end
end
        
    