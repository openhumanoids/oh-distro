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
    fc_publish_period = 0.01;
    
    % Atlas, for usefulness
    r;
    r_control;
    
    % Whether we should publish a ground truth EST_ROBOT_STATE
    % (or if not, publish approp messages to feed state_sync state est.)
    publish_truth = 1;
    
    % reordering to generate atlas_state_t messages for state est
    reordering;
    
  end
  
  methods
    function obj = LCMBroadcastBlock(r,r_control,options)
      typecheck(r,'Atlas');
      if (nargin >= 2 && ~isempty(r_control))
        typecheck(r_control, 'Atlas');
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
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate
      
      if isfield(options,'publish_truth')
        obj.publish_truth = options.publish_truth;
      end
      
      % Get LCM set up for broadcast on approp channels
      obj.lc = lcm.lcm.LCM.getSingleton();
      
      atlascoordnames = obj.getInputFrame.getFrameByName('AtlasState').getCoordinateNames;
      atlascoordnames = atlascoordnames(7:end); % cut off the floating base
      if (r.hands>0)
        hand_names = obj.getInputFrame.getFrameByName('HandState').getCoordinateNames;
        names = [atlascoordnames;
                hand_names(1:30)];
        % all of the hand names should be prepended as being right hand
        for i=29:length(names)
          names{i} = ['right_', names{i}];
        end
      else
        names = [atlascoordnames];
      end
      % Append hokuyo_joint if we're not going to have state_sync help with
      % this
      if (obj.publish_truth)
        names = [names; ;
                'hokuyo_joint'];
      end
      obj.joint_names_cache = cell(length(names), 1);
      % (this is the part that we really don't want to run repeatedly...)
      for i=1:length(names)
        obj.joint_names_cache(i) = java.lang.String(names(i));
      end
      
      % figure out joint reordering -- atlas_state_t assumes joints
      % in a particular order :P
      desired_joint_name_order = {'back_bkz',
                                  'back_bky',
                                  'back_bkx',
                                  'neck_ay',
                                  'l_leg_hpz',
                                  'l_leg_hpx',
                                  'l_leg_hpy',
                                  'l_leg_kny',
                                  'l_leg_aky',
                                  'l_leg_akx',
                                  'r_leg_hpz',
                                  'r_leg_hpx',
                                  'r_leg_hpy',
                                  'r_leg_kny',
                                  'r_leg_aky',
                                  'r_leg_akx',
                                  'l_arm_usy',
                                  'l_arm_shx',
                                  'l_arm_ely',
                                  'l_arm_elx',
                                  'l_arm_uwy',
                                  'l_arm_mwx',
                                  'r_arm_usy',
                                  'r_arm_shx',
                                  'r_arm_ely',
                                  'r_arm_elx',
                                  'r_arm_uwy',
                                  'r_arm_mwx'
                                  };
      obj.reordering = zeros(length(desired_joint_name_order), 1);
      for i=1:length(desired_joint_name_order)
        obj.reordering(i) = find(strcmp(desired_joint_name_order{i}, atlascoordnames));
      end
      
      obj.r = r;
      obj.r_control = r_control;
    end
    
    function varargout=mimoOutput(obj,t,~,varargin)
      inp = obj.getInputFrame();
      num = inp.getFrameNumByName('AtlasState');
      if length(num)~=1
        error(['No atlas state found as input for LCMBroadcastBlock!']);
      end
      atlas_state = varargin{num};
      
      num = inp.getFrameNumByName('HandState');
      hand_state = [];
      if (length(num)>1)
        error(['Ambiguous hand state. No support for two hands yet...']);
      elseif (length(num)==1)
        hand_state = varargin{num};
      end
      
      num = inp.getFrameNumByName('hokuyo');
      laser_state = [];
      if (length(num)>1)
        error(['Ambiguous hand state. No support for two hands yet...']);
      elseif (length(num)==1)
        laser_state = varargin{num};
      end
      
      if (~isempty(laser_state))
        laser_spindle_angle = laser_state(1)+pi/2;
        laser_ranges = laser_state(2:end);
      else
        laser_spindle_angle = 0;
        laser_ranges = [];
      end
      
      % See if we just passed our publish-timestep for 
      % the foot contact state message, publish if so (and if
      % we're publishing ground truth anyway)
      if (obj.publish_truth && mod(t, obj.fc_publish_period)  < obj.r.timestep)
        % Get foot force state
%         num = inp.getFrameNumByName('ForceTorque');
%         hand_state = [];
%         if (length(num)~=2)
%           lfoot_force = [];
%           rfoot_force = [];
%         else
%           lfoot_force = varargin{num(1)};
%           rfoot_force = varargin{num(2)};
%         end;
%         fc = [norm(lfoot_force); norm(rfoot_force)];
        % Get binary foot contact, call it force:
        x = atlas_state;
        [phiC,~,~,~,~,idxA,idxB,~,~,~] = obj.r_control.getManipulator().contactConstraints(x(1:length(x)/2),false);
        within_thresh = phiC < 0.002;
        contact_pairs = [idxA(within_thresh) idxB(within_thresh)];
        fc = [any(any(contact_pairs == obj.r_control.findLinkInd('l_foot')));
              any(any(contact_pairs == obj.r_control.findLinkInd('r_foot')))];
       
        % Publish it!
        foot_contact_est = drc.foot_contact_estimate_t();
        foot_contact_est.utime = t*1000*1000;
        foot_contact_est.left_contact = fc(1);
        foot_contact_est.right_contact = fc(2);
        foot_contact_est.detection_method = 0;
        obj.lc.publish('FOOT_CONTACT_ESTIMATE', foot_contact_est);
      end
      

      
      % What needs to go out:
      num_dofs = length([atlas_state; hand_state]) / 2;
      atlas_dofs = length(atlas_state)/2;
      hand_dofs = length(hand_state)/2;
      % Robot state publishing (channel depends on whether we're publishing
      % truth (on EST_ROBOT_STATE) or not (ATLAS_STATE)
      if (~obj.publish_truth)
        state_msg = drc.atlas_state_t();
      else
        state_msg = drc.robot_state_t();
      end
      
      state_msg.utime = t*1000*1000;
      
      if (obj.publish_truth)
        % ATLAS_STATE has no global pose, but EST_ROBOT_STATE does
        state_msg.pose = drc.position_3d_t();
        state_msg.pose.translation = drc.vector_3d_t();
        state_msg.pose.rotation = drc.quaternion_t();
        state_msg.pose.translation.x = atlas_state(1);
        state_msg.pose.translation.y = atlas_state(2);
        state_msg.pose.translation.z = atlas_state(3);

        q = rpy2quat([atlas_state(4) atlas_state(5) atlas_state(6)]);
        state_msg.pose.rotation.w = q(1);
        state_msg.pose.rotation.x = q(2);
        state_msg.pose.rotation.y = q(3);
        state_msg.pose.rotation.z = q(4);

        state_msg.twist = drc.twist_t();
        state_msg.twist.linear_velocity = drc.vector_3d_t();
        state_msg.twist.angular_velocity = drc.vector_3d_t();
        state_msg.twist.linear_velocity.x = atlas_state(atlas_dofs+1);
        state_msg.twist.linear_velocity.y = atlas_state(atlas_dofs+2);
        state_msg.twist.linear_velocity.z = atlas_state(atlas_dofs+3);
        state_msg.twist.angular_velocity.x = atlas_state(atlas_dofs+4);
        state_msg.twist.angular_velocity.y = atlas_state(atlas_dofs+5);
        state_msg.twist.angular_velocity.z = atlas_state(atlas_dofs+6);
      end
      
      if (obj.publish_truth)
        % will publish the multisense angle as part of total
        % est_robot_state
        state_msg.num_joints = num_dofs-6+1;
        % only est_robot_state atlas_state message has joint names
        state_msg.joint_name = obj.joint_names_cache;
      else
        state_msg.num_joints = num_dofs-6;
      end
      
      state_msg.joint_position=zeros(1,state_msg.num_joints);
      state_msg.joint_velocity=zeros(1,state_msg.num_joints);
      state_msg.joint_effort=zeros(1,state_msg.num_joints);
        
      atlas_pos = atlas_state(7:atlas_dofs);
      atlas_vel = atlas_state(atlas_dofs+7:end);
      if (~obj.publish_truth)
        atlas_pos = atlas_pos(obj.reordering);
        atlas_vel = atlas_vel(obj.reordering);
         state_msg.joint_position = [atlas_pos; hand_state(1:hand_dofs)];
        state_msg.joint_velocity = [atlas_vel; hand_state(hand_dofs+1:end)];
      else
        % need another factor of pi/2though I wouldn't mind it being earlier rather than later to get drawn multisense to agree with
        % the "true" (functionally true, anyway) mirror position
        state_msg.joint_position = [atlas_pos; hand_state(1:hand_dofs); laser_spindle_angle+pi/2];
        state_msg.joint_velocity = [atlas_vel; hand_state(hand_dofs+1:end); 0];
      end
      state_msg.force_torque = drc.force_torque_t();
      % if we're publishing to satisfy state est we need force sensing from feet
%       if (~obj.publish_truth)
%         % Get binary foot contact, call it force:
%         x = atlas_state;
%         [phiC,~,~,~,~,idxA,idxB,~,~,~] = obj.r_control.getManipulator().contactConstraints(x(1:length(x)/2),false);
%         within_thresh = phiC < 0.002;
%         contact_pairs = [idxA(within_thresh) idxB(within_thresh)];
%         fc = [any(any(contact_pairs == obj.r_control.findLinkInd('l_foot')));
%           any(any(contact_pairs == obj.r_control.findLinkInd('r_foot')))];
%         % pack it up
%         state_msg.force_torque.l_foot_force_z = fc(1)*1000;
%         state_msg.force_torque.r_foot_force_z = fc(1)*1000;
%       end
      
      if (~obj.publish_truth)
        obj.lc.publish('ATLAS_STATE', state_msg);
      else
        obj.lc.publish('EST_ROBOT_STATE', state_msg);
      end
      
      % force an update of the body pose too
      % (shouldn't this be being inferred from EST_ROBOT_STATE?)
      pose_body_frame = bot_core.pose_t();
      pose_body_frame.utime = t*1000*1000;
      pose_body_frame.pos = [atlas_state(1), atlas_state(2), atlas_state(3)];
      pose_body_frame.vel = [atlas_state(1+atlas_dofs), atlas_state(2+atlas_dofs), atlas_state(3+atlas_dofs)];
      q = rpy2quat([atlas_state(4) atlas_state(5) atlas_state(6)]);
      pose_body_frame.vel = quatRotateVec(q, pose_body_frame.vel);
      pose_body_frame.orientation = [q(1) q(2) q(3) q(4)]; % Rotation
      pose_body_frame.rotation_rate = [atlas_state(4+atlas_dofs) atlas_state(5+atlas_dofs) atlas_state(6+atlas_dofs)];
      pose_body_frame.accel = [0 0 0];
      if (obj.publish_truth)
        obj.lc.publish('POSE_BODY', pose_body_frame);
      else
        obj.lc.publish('POSE_BDI', pose_body_frame);
      end
      
      
      % Send over
      % -- "MULTISENSE_STATE" -- lcm_state msg for joint hokuyo_joint,
      %     labeled with tiem of scan
      % -- also "PRE_SPINDLE_TO_POST_SPINDLE" with the approp transform
      % -- To channel "SCAN", publish populated lcm_laser_msg
      if (~isempty(laser_state))
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
        
        multisense_state.joint_position = [mod(laser_spindle_angle+pi/2, 2*pi), ...
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
        q = rpy2quat([0, 0, laser_spindle_angle]);
        pre_to_post_frame.quat = [q(1) q(2) q(3) q(4)]; % Rotation
        obj.lc.publish('PRE_SPINDLE_TO_POST_SPINDLE', pre_to_post_frame);
        
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
        lcm_laser_msg.radstep = obj.hokuyo_yaw_width / (length(laser_ranges) - 1);
        obj.lc.publish('SCAN', lcm_laser_msg);
        
      end
      varargout = varargin;
      
    end
  end
  
end
