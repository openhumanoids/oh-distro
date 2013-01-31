classdef ClosedGraspControllerInProg < DrakeSystem
   % Controller for sandia_hand opening and closing, input is on/off signal and
   % output is desired joint angles
   
   % remember for the quat2rpy function the inputs go as follows: w=q(1);x=q(2);y=q(3);z=q(4);
   
    methods
        function obj = ClosedGraspControllerInProg(r)
            typecheck(r,'TimeSteppingRigidBodyManipulator');
            numOutputs = r.getNumStates/2;
            
            obj=obj@DrakeSystem(0,1,19,numOutputs);  

            frame_in = GraspEEGoalCmd(r);
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
%             u(1) = x; u(2) = y; u(3) = z;
%             u(4) = quat x; u(5) = quat_y; u(6) = quat_z; u(7) = quat_w;
%             rest of u's are joints
            x0 = Point(obj.getOutputFrame);
            
            x0.real_base_x = u(1);
            x0.real_base_y = u(2);
            x0.real_base_z = u(3);
            quat = [u(7);u(4);u(5);u(6)];
            rpy = quat2rpy(quat);
            x0.real_base_roll = rpy(1);
            x0.real_base_pitch = rpy(2);
            x0.real_base_yaw = rpy(3);
            
            x0.right_f0_j0= u(8);
            x0.right_f0_j1= u(9);
            x0.right_f0_j2= u(10);

            x0.right_f1_j0= u(11);
            x0.right_f1_j1= u(12);
            x0.right_f1_j2= u(13);

            x0.right_f2_j0= u(14);
            x0.right_f2_j1= u(15);
            x0.right_f2_j2= u(16);
            
            x0.right_f3_j0= u(17);
            x0.right_f3_j1= u(18);
            x0.right_f3_j2= u(19);
            
            y=double(x0);
        
        end
    end
end
            
        