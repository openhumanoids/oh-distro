classdef LCMConstOut < DrakeSystem
    
    methods
        function obj = LCMConstOut()
            obj=obj@DrakeSystem(0,0,1,1); %No states and SISO
            out_frame=GraspCmd();
            obj=setOutputFrame(obj,out_frame);
        end
        
        function y=output(obj,t,x,u)
            y = 1;
        end
    end
end