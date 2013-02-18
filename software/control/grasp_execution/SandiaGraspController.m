classdef SandiaGraspController < MIMODrakeSystem
    % A grasp controller that runs on the robot side. S
    % structured to be able to handle bi-handed grasps for the debris removal task.
    
    properties
        l_manip
        r_manip
    end
    
    methods
        function obj = SandiaGraspController(r_left,r_right)
            % @param r_left the time-stepping manipulator for left hand
            % @param r_right the time-stepping manipulator for right hand
            typecheck(r_left,'TimeSteppingRigidBodyManipulator');
            typecheck(r_right,'TimeSteppingRigidBodyManipulator');
            l_numOutputs = r_left.getNumStates/2;
            r_numOutputs = r_right.getNumStates/2;
            
            
            % Define Frames
            input_frame = GraspState(r_left,r_right);
            
            left_joint_cmd_out_frame = SandiaJointCommand(r_left,'left');
            right_joint_cmd_out_frame = SandiaJointCommand(r_right,'right');
            output_frame = MultiCoordinateFrame({left_joint_cmd_out_frame,right_joint_cmd_out_frame});
            
            
            %dummy state for now, all the logic is in the output function
            % eventually x will be l/r palm poses and measured tactile feedback.
            num_xd = 0;
            obj = obj@MIMODrakeSystem(0,num_xd,input_frame,output_frame,true,true);
            %obj = setSampleTime(obj,[.01;0]); % sets controller update rate
            obj = setInputFrame(obj,input_frame);
            obj = setOutputFrame(obj,output_frame);
            obj.l_manip = r_left;
            obj.r_manip = r_right;
            
        end
        
        function xdn = mimoUpdate(obj,t,x,varargin)
            % u is grasp state
            u = varargin{1};
        end
        
        function varargout = mimoOutput(obj,t,x,varargin)
            u = varargin{1};
            nq_l = obj.l_manip.getNumStates()/2;
            nq_r = obj.r_manip.getNumStates()/2;
            
            %%%   manual encoding
            %   n_l_joints =nq_l-6;
            %   n_r_joints =nq_r-6;
            %   l_joint_positions = u(19:19+n_l_joints-1);
            %   r_joint_positions = u(19+n_l_joints:19+n_l_joints+n_r_joints-1);
            %   [r,p,y] = quat2angle(u(6:9)','ZYX');
            %   y_l = [u(3:5);r;p;y;l_joint_positions];
            %   [r,p,y] = quat2angle(u(13:16)','ZYX');
            %   y_r = [u(10:12);r;p;y;r_joint_positions];
            
            msg = obj.getInputFrame.lcmcoder.encode(t,u);
            [r,p,y] = quat2angle(u(6:9)','ZYX');
            q_l = [u(3:5);r;p;y;msg.l_joint_position];
            [r,p,y] = quat2angle(u(13:16)','ZYX');
            q_r = [u(10:12);r;p;y;msg.r_joint_position];
            if(msg.grasp_type==msg.SANDIA_LEFT)
                publish(obj.getOutputFrame.frame{1},t,q_l,'L_HAND_JOINT_COMMANDS');
            elseif(msg.grasp_type==msg.SANDIA_RIGHT)
                publish(obj.getOutputFrame.frame{2},t,q_r,'R_HAND_JOINT_COMMANDS');
            end
            % Publishing manually as MultiCoordinateFrame is not considered a LCMPublisher.
            varargout = {q_l,q_r}; % Q: does this publish both left and right joint commands all the time? What if you need to supress one or the other?
        end
        
        
    end %% end methods
    
end

