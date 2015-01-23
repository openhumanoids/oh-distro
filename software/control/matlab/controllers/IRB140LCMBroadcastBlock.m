classdef IRB140LCMBroadcastBlock < MIMODrakeSystem
  
  properties
    lc; % LCM
    joint_names_cache;
    r_hand_joint_names_cache;
    r_hand_joint_inds;
    
    % robot
    r;
    r_control;
    nq_control;
    
    % Structure containing the frame numbers of the different states
    % inside the input frame.
    frame_nums;
  end
  
  methods
    function obj = IRB140LCMBroadcastBlock(r,r_control,options)
      typecheck(r,'IRB140');
      if (nargin >= 2 && ~isempty(r_control))
        typecheck(r_control, 'IRB140');
      else
        r_control = r;
      end
      
      if nargin<3
        options = struct();
      end
      
      input_frame = getOutputFrame(r);
      output_frame = getOutputFrame(r);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.001;
      end
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate

      % Get LCM set up for broadcast on approp channels
      obj.lc = lcm.lcm.LCM.getSingleton();
      
      if (isa(obj.getInputFrame, 'IRB140State'))
        coordnames = obj.getInputFrame.getCoordinateNames;
      else
        coordnames = obj.getInputFrame.getFrameByName('IRB140State').getCoordinateNames;
      end
      
      % write down names for the hands. these differ from simul a bit
      % (we have the fourbar explicitely), so I'm doing this by hand...
      if (r.hands>0)
        hand_names = {'left_finger_1_joint_1',
                      'left_finger_1_joint_2',
                      'left_finger_1_joint_3',
                      'left_finger_2_joint_1',
                      'left_finger_2_joint_2',
                      'left_finger_2_joint_3',
                      'left_finger_middle_joint_1',
                      'left_finger_middle_joint_2',
                      'left_finger_middle_joint_3',
                      'left_palm_finger_1_joint',
                      'left_palm_finger_2_joint'
                      };
        obj.r_hand_joint_names_cache = cell(11, 1);
        for i=1:length(hand_names)
          obj.r_hand_joint_names_cache(i) = java.lang.String(hand_names{i});
        end
        % not sure what the last two correspond to. for now I'm going to 
        % zero them.
        obj.r_hand_joint_inds = [1, 2, 3, 11, 12, 13, 21, 22, 23, 10, 20];
      end
      
      obj.joint_names_cache = cell(length(coordnames)/2, 1);
      % (this is the part that we really don't want to run repeatedly...)
      for i=1:length(coordnames)/2
        obj.joint_names_cache(i) = java.lang.String(coordnames(i));
      end
      
      if (isa(input_frame, 'MultiCoordinateFrame'))
        obj.frame_nums.irb140_state = input_frame.getFrameNumByName('IRB140State');
        obj.frame_nums.hand_state = input_frame.getFrameNumByName('drcFrames.HandState');
      else
        obj.frame_nums.irb140_state = 1;
        obj.frame_nums.hand_state = '';
      end
      
      obj.r = r;
      obj.r_control = r_control;
      obj.nq_control = obj.r_control.getNumPositions();
    end
    
    function varargout=mimoOutput(obj,t,~,varargin)
      irb140_state = varargin{obj.frame_nums.irb140_state};
      hand_state = [];
      if (obj.frame_nums.hand_state)
        hand_state_ours = varargin{obj.frame_nums.hand_state};
        % Map it to the hand state the rest of the system understands
        hand_state = [hand_state_ours(obj.r_hand_joint_inds);
                      hand_state_ours(length(hand_state_ours)/2+obj.r_hand_joint_inds)];
        hand_state(10:11) = [0;0];
      end
      
      % What needs to go out:
      num_dofs = length([irb140_state; hand_state]) / 2;
      irb140_dofs = length(irb140_state)/2;
      hand_dofs = length(hand_state)/2;
      
      state_msg = drc.robot_state_t();
      state_msg.utime = t*1000*1000;
      
      state_msg.pose = drc.position_3d_t();
      state_msg.pose.translation = drc.vector_3d_t();
      state_msg.pose.rotation = drc.quaternion_t();
      state_msg.pose.translation.x = obj.r.base_offset(1);
      state_msg.pose.translation.y = obj.r.base_offset(2);
      state_msg.pose.translation.z = obj.r.base_offset(3);

      q = rpy2quat(obj.r.base_rpy);
      state_msg.pose.rotation.w = q(1);
      state_msg.pose.rotation.x = q(2);
      state_msg.pose.rotation.y = q(3);
      state_msg.pose.rotation.z = q(4);

      state_msg.twist = drc.twist_t();
      state_msg.twist.linear_velocity = drc.vector_3d_t();
      state_msg.twist.angular_velocity = drc.vector_3d_t();
      state_msg.twist.linear_velocity.x = 0;
      state_msg.twist.linear_velocity.y = 0;
      state_msg.twist.linear_velocity.z = 0;
      state_msg.twist.angular_velocity.x = 0;
      state_msg.twist.angular_velocity.y = 0;
      state_msg.twist.angular_velocity.z = 0;
      
      state_msg.num_joints = num_dofs;
      state_msg.joint_name = [obj.joint_names_cache; obj.r_hand_joint_names_cache];
      
      state_msg.joint_position=zeros(1,state_msg.num_joints);
      state_msg.joint_velocity=zeros(1,state_msg.num_joints);
      state_msg.joint_effort=zeros(1,state_msg.num_joints);
        
      irb140_pos = irb140_state(1:irb140_dofs);
      irb140_vel = irb140_state(irb140_dofs+1:end);
     
      state_msg.joint_position = [irb140_pos; hand_state(1:hand_dofs)];
      state_msg.joint_velocity = [irb140_vel; hand_state(hand_dofs+1:end)];
        
      state_msg.force_torque = drc.force_torque_t();

      obj.lc.publish('EST_ROBOT_STATE', state_msg);
      
      varargout = varargin;
      
    end
  end
  
end
