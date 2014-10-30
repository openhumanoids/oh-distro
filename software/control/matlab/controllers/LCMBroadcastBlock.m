classdef LCMBroadcastBlock < MIMODrakeSystem
  
  properties
    lc; % LCM
    spindle_channel;
    hokuyo_data_channel;
    robot_state_est_channel;
    hokuyo_yaw_width;
    hokuyo_num_pts;
    hokuyo_max_range;
    hokuyo_spin_rate;
    joint_names_cache;
    
    % FC publish period
    fc_publish_period = 0.1;
    
    % Atlas, for usefulness
    r;
  end
  
  methods
    function obj = LCMBroadcastBlock(r,options)
      typecheck(r,'Biped');
      
      if nargin<2
        options = struct();
      end
      
      input_frame = getOutputFrame(r);
      output_frame = getOutputFrame(r);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      obj.hokuyo_yaw_width = r.hokuyo_yaw_width;
      obj.hokuyo_num_pts = r.hokuyo_num_pts;
      obj.hokuyo_max_range = r.hokuyo_max_range;
      obj.hokuyo_spin_rate = r.hokuyo_spin_rate;
      
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.001;
      end
      obj = setSampleTime(obj,[0.01;0]); % sets controller update rate
      
      % Get LCM set up for broadcast on approp channels
      obj.lc = lcm.lcm.LCM.getSingleton();
      
      names = obj.getInputFrame.getFrameByName('AtlasState').getCoordinateNames;
      obj.joint_names_cache = cell(length(names), 1);
      for i=1:length(names)
        obj.joint_names_cache(i) = java.lang.String(names(i));
      end
      
      obj.r = r;
    end
    
    function varargout=mimoOutput(obj,t,~,varargin)
      % See if we just passed our publish-timestep for 
      % the foot contact state message, publish if so.
      if (mod(t, obj.fc_publish_period)  < obj.r.timestep)
        % Get foot contact state
        x = varargin{1};
        [phiC,~,~,~,~,idxA,idxB,~,~,~] = obj.r.getManipulator().contactConstraints(x(1:length(x)/2),false);
        within_thresh = phiC < 0.002;
        contact_pairs = [idxA(within_thresh) idxB(within_thresh)];
        fc = [any(any(contact_pairs == obj.r.findLinkInd('l_foot')));
              any(any(contact_pairs == obj.r.findLinkInd('r_foot')))];
        % Publish it!
        foot_contact_est = drc.foot_contact_estimate_t();
        foot_contact_est.utime = t*1000*1000;
        foot_contact_est.left_contact = fc(1);
        foot_contact_est.right_contact = fc(2);
        foot_contact_est.detection_method = 0;
        obj.lc.publish('FOOT_CONTACT_ESTIMATE', foot_contact_est);
      end
      
      % What needs to go out:
      num_dofs = length(varargin{1}) / 2;
      % Robot state on EST_ROBOT_STATE.
      state_msg = drc.robot_state_t();
      state_msg.utime = t*1000*1000;
      
      state_msg.pose = drc.position_3d_t();
      state_msg.pose.translation = drc.vector_3d_t();
      state_msg.pose.rotation = drc.quaternion_t();
      state_msg.pose.translation.x = varargin{1}(1);
      state_msg.pose.translation.y = varargin{1}(2);
      state_msg.pose.translation.z = varargin{1}(3);
      
      q = rpy2quat([varargin{1}(4) varargin{1}(5) varargin{1}(6)]);
      state_msg.pose.rotation.w = q(1);
      state_msg.pose.rotation.x = q(2);
      state_msg.pose.rotation.y = q(3);
      state_msg.pose.rotation.z = q(4);
      
      state_msg.twist = drc.twist_t();
      state_msg.twist.linear_velocity = drc.vector_3d_t();
      state_msg.twist.angular_velocity = drc.vector_3d_t();
      state_msg.twist.linear_velocity.x = varargin{1}(num_dofs+1);
      state_msg.twist.linear_velocity.y = varargin{1}(num_dofs+2);
      state_msg.twist.linear_velocity.z = varargin{1}(num_dofs+3);
      state_msg.twist.angular_velocity.x = varargin{1}(num_dofs+4);
      state_msg.twist.angular_velocity.y = varargin{1}(num_dofs+5);
      state_msg.twist.angular_velocity.z = varargin{1}(num_dofs+6);
      
      state_msg.num_joints = num_dofs;
      state_msg.joint_name=javaArray('java.lang.String', state_msg.num_joints);
      state_msg.joint_position=zeros(1,state_msg.num_joints);
      state_msg.joint_velocity=zeros(1,state_msg.num_joints);
      state_msg.joint_effort=zeros(1,state_msg.num_joints);
      state_msg.joint_name = obj.joint_names_cache;
      state_msg.joint_position = varargin{1}(1:num_dofs);
      state_msg.joint_velocity = varargin{1}(num_dofs+1:2*num_dofs);
      state_msg.force_torque = drc.force_torque_t();
      obj.lc.publish('EST_ROBOT_STATE', state_msg);
      
      % force an update of the body pose too
      % (shouldn't this be being inferred from EST_ROBOT_STATE?)
      
      pose_body_frame = bot_core.pose_t();
      pose_body_frame.utime = t*1000*1000;
      pose_body_frame.pos = [varargin{1}(1), varargin{1}(2), varargin{1}(3)];
      pose_body_frame.vel = [varargin{1}(1+num_dofs), varargin{1}(2+num_dofs), varargin{1}(3+num_dofs)];
      q = rpy2quat([varargin{1}(4) varargin{1}(5) varargin{1}(6)]);
      pose_body_frame.orientation = [q(1) q(2) q(3) q(4)]; % Rotation
      pose_body_frame.rotation_rate = [varargin{1}(4++num_dofs) varargin{1}(5++num_dofs) varargin{1}(6++num_dofs)];
      pose_body_frame.accel = [0 0 0];
      obj.lc.publish('POSE_BODY', pose_body_frame);
      
      % Send over
      % -- "MULTISENSE_STATE" -- lcm_state msg for joint hokuyo_joint,
      %     labeled with tiem of scan
      % -- also "PRE_SPINDLE_TO_POST_SPINDLE" with the approp transform
      % -- also "POST_SPINDLE_TO_SCAN" to adjust for our different spindle
      % setup
      % -- To channel "SCAN", publish populated lcm_laser_msg
      
      if length(varargin) > 1
        laser_spindle_angle = varargin{2}(1);
        laser_ranges = varargin{2}(2:end);
        % MULTISENSE_STATE and PRE_SPINDLE_TO_POST_SPINDLE, beginning of scan
        multisense_state = multisense.state_t();
        multisense_state.joint_name = {'hokuyo_joint',
          'pre_spindle_cal_x_joint',
          'pre_spindle_cal_y_joint',
          'pre_spindle_cal_z_joint',
          'pre_spindle_cal_roll_joint',
          'pre_spindle_cal_pitch_joint',
          'pre_spindle_cal_yaw_joint',
          'post_spindle_cal_x_joint',
          'post_spindle_cal_y_joint',
          'post_spindle_cal_z_joint',
          'post_spindle_cal_roll_joint',
          'post_spindle_cal_pitch_joint',
          'post_spindle_cal_yaw_joint'};
        
        multisense_state.joint_position = [laser_spindle_angle, ...
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        multisense_state.joint_velocity = [0.0, ...
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        multisense_state.joint_effort = [0.0, ...
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        multisense_state.num_joints = length(multisense_state.joint_name);
        multisense_state.utime = t*1000*1000;
        obj.lc.publish('MULTISENSE_STATE', multisense_state);
        
        pre_to_post_frame = bot_core.rigid_transform_t();
        pre_to_post_frame.utime = t*1000*1000;
        pre_to_post_frame.trans = [0, 0, 0]; % no offset
        q = rpy2quat([0, 0, laser_spindle_angle+pi]);
        pre_to_post_frame.quat = [q(1) q(2) q(3) q(4)]; % Rotation
        obj.lc.publish('PRE_SPINDLE_TO_POST_SPINDLE', pre_to_post_frame);
        
        % Our spindle setup has the laser perfectly centered on the
        % spindle
        post_to_scan_frame = pre_to_post_frame;
        post_to_scan_frame.trans = [ 0,0,0 ];
        q = rpy2quat([0, -90, -90]*pi/180);
        post_to_scan_frame.quat = [q(1) q(2) q(3) q(4)]; % Rotation
        obj.lc.publish('POST_SPINDLE_TO_SCAN', post_to_scan_frame);
        
        % And the data
        lcm_laser_msg = bot_core.planar_lidar_t();
        lcm_laser_msg.utime = t*1000*1000;
        lcm_laser_msg.ranges = laser_ranges;
        % just setting intensities high enough that the points aren't
        % discarded for now...
        lcm_laser_msg.intensities = 5000*ones(size(laser_ranges));
        lcm_laser_msg.nranges = length(laser_ranges);
        lcm_laser_msg.nintensities = length(laser_ranges);
        lcm_laser_msg.rad0 = -obj.hokuyo_yaw_width/2.0;
        lcm_laser_msg.radstep = obj.hokuyo_yaw_width / (length(laser_ranges)-1);
        obj.lc.publish('SCAN', lcm_laser_msg);
        
      end
      varargout = varargin;
      
    end
  end
  
end
