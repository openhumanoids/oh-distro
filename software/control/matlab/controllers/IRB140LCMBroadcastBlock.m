classdef IRB140LCMBroadcastBlock < MIMODrakeSystem
  
  properties
    lc; % LCM
    joint_names_cache;
    
    % robot
    r;
    r_control;
    nq_control;
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
      
      if (isa(obj.getInputFrame, 'drcFrames.IRB140State'))
        coordnames = obj.getInputFrame.getCoordinateNames;
      else
        coordnames = obj.getInputFrame.getFrameByName('drcFrames.IRB140State').getCoordinateNames;
      end

      obj.joint_names_cache = cell(length(coordnames), 1);
      % (this is the part that we really don't want to run repeatedly...)
      for i=1:length(coordnames)
        obj.joint_names_cache(i) = java.lang.String(coordnames(i));
      end
      
      obj.r = r;
      obj.r_control = r_control;
      obj.nq_control = obj.r_control.getNumPositions();
    end
    
    function varargout=mimoOutput(obj,t,~,varargin)
      irb140_state = varargin{1};
      
      % What needs to go out:
      irb140_dofs = length(irb140_state)/2;
      
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
      
      state_msg.num_joints = irb140_dofs;
      state_msg.joint_name = [obj.joint_names_cache];
      
      state_msg.joint_position=zeros(1,state_msg.num_joints);
      state_msg.joint_velocity=zeros(1,state_msg.num_joints);
      state_msg.joint_effort=zeros(1,state_msg.num_joints);
        
      irb140_pos = irb140_state(1:irb140_dofs);
      irb140_vel = irb140_state(irb140_dofs+1:end);
     
      state_msg.joint_position = irb140_pos;
      state_msg.joint_velocity = irb140_vel;
        
      state_msg.force_torque = drc.force_torque_t();

      obj.lc.publish('EST_ROBOT_STATE', state_msg);
      
      varargout = varargin;
      
    end
  end
  
end
