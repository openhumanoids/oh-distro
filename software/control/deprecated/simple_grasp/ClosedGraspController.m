classdef ClosedGraspController < DrakeSystem
   % Controller for sandia_hand opening and closing, input is on/off signal and
   % output is desired joint angles
    methods
        function obj = ClosedGraspController(r)
            typecheck(r,'TimeSteppingRigidBodyManipulator');
            numOutputs = r.getNumStates/2;
            
            obj=obj@DrakeSystem(0,1,1,numOutputs);  

            frame_in = GraspCmd();
            frame_out = SandiaPositionRef(r);
            
            obj=setInputFrame(obj,frame_in);
            obj=setOutputFrame(obj,frame_out);  
            obj = setSampleTime(obj,[.01;0]);
        end          
        
        function x_dn = update(obj,t,u,x)
            x_dn = 1;
        end
        
        function x_dn = getInitialState(obj,t,u,x)
            x_dn = 1;
        end
          
        function y = output(obj,t,x,u)
            if u > .5
                x0 = Point(obj.getOutputFrame);
                
                x0.real_base_x = .1;
                x0.real_base_y = .15;
                x0.real_base_z = .3;
                x0.real_base_roll = 1.7;
%                 x0.real_base_pitch = 1.57;
%                 x0.real_base_yaw = 1;
                
                x0.right_f3_j0=-0.33;
                x0.right_f3_j1=0.46;
                x0.right_f3_j2=1.57;

                x0.right_f2_j1=1.57;
                x0.right_f2_j2=1.57;

                x0.right_f1_j1=1.57;
                x0.right_f1_j2=1.57;

                x0.right_f0_j1=1.57;
                x0.right_f0_j2=1.57;
                
%                 x0 = obj.manip.resolveConstraints(double(x0)); %
%                 Uncommenting this line seems to yield an error that I
%                 don't quite understand, refer to Jan 16 in UROP 2013
%                 google doc for more details

                y=double(x0);
                
            else
                x0 = Point(obj.getOutputFrame); 
                x0.real_base_x = .1;
                x0.real_base_y = .15;
                x0.real_base_z = .3;
                x0.real_base_roll = 1.7;
%                 x0.real_base_pitch = 1.57;
%                 x0.real_base_yaw = 1;
                y=double(x0);
            end
        end
    end
end
            
        