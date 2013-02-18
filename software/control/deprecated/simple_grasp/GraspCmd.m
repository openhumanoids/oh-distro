classdef GraspCmd < LCMCoordinateFrameWCoder & Singleton
    
    methods
        function obj = GraspCmd()
            coder = GraspCmdCoder('sandia_hand');
            obj = obj@LCMCoordinateFrameWCoder('grasp_cmd',1,'i',JLCMCoder(coder));
            obj.setDefaultChannel('GRASP_CMD')
        end
    end
end