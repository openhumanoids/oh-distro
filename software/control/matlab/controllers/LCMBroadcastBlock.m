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
    r_hand_joint_names_cache;
    r_hand_joint_inds;
    l_hand_joint_names_cache;
    l_hand_joint_inds;
    r_foot_id
    l_foot_id
    % FC publish period
    fc_publish_period = 0.01;
    
    % Atlas, for usefulness
    r;
    r_control;
    nq_control;

    % Structure containing the frame numbers of the different states
    % inside the input frame.
    frame_nums;
    
    % Whether we should publish a ground truth EST_ROBOT_STATE
    % (or if not, publish approp messages to feed state_sync state est.)
    publish_truth = 1;
    
    % Publishing IMU?
    publish_imu = 0;
    
    % reordering to generate atlas_state_t messages for state est
    reordering;
    
    % Shared data handle for storing the state from the last step
    % (quick hack to check out if this kind of numerical imu approx might start
    % to work here...)
    last_floating_state = SharedDataHandle(zeros(12, 1));
    
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
      
      if isfield(options,'publish_imu')
        obj.publish_imu = options.publish_imu;
      end
      
      % Get LCM set up for broadcast on approp channels
      obj.lc = lcm.lcm.LCM.getSingleton();
      
      if (isa(obj.getInputFrame, 'drcFrames.AtlasState'))
        atlascoordnames = obj.getInputFrame.getCoordinateNames;
      else
        atlascoordnames = obj.getInputFrame.getFrameByNameRecursive('drcFrames.AtlasState').getCoordinateNames;
      end
      atlascoordnames = atlascoordnames(7:length(atlascoordnames)/2); % cut off the floating base

      obj.joint_names_cache = cell(length(atlascoordnames), 1);
      % (this is the part that we really don't want to run repeatedly...)
      for i=1:length(atlascoordnames)
        obj.joint_names_cache(i) = java.lang.String(atlascoordnames(i));
      end
      % write down names for the hands. these differ from simul a bit
      % (we have the fourbar explicitely), so I'm doing this by hand...
      if (r.hand_right > 0)
        r_hand_names = {'right_finger_1_joint_1',
                      'right_finger_1_joint_2',
                      'right_finger_1_joint_3',
                      'right_finger_2_joint_1',
                      'right_finger_2_joint_2',
                      'right_finger_2_joint_3',
                      'right_finger_middle_joint_1',
                      'right_finger_middle_joint_2',
                      'right_finger_middle_joint_3',
                      'right_palm_finger_1_joint',
                      'right_palm_finger_2_joint'
                      };
        obj.r_hand_joint_names_cache = cell(11, 1);
        for i=1:length(r_hand_names)
          obj.r_hand_joint_names_cache(i) = java.lang.String(r_hand_names{i});
        end
        % not sure what the last two correspond to. for now I'm going to 
        % zero them.
        if (strcmp(r.hand_right_kind, 'robotiq'))
          obj.r_hand_joint_inds = [1, 2, 3, 11, 12, 13, 21, 22, 23, 10, 20];
        elseif (strcmp(r.hand_right_kind, 'robotiq_tendons'))
          obj.r_hand_joint_inds = [1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 1];
        elseif (strcmp(r.hand_right_kind, 'robotiq_simple'))
          obj.r_hand_joint_inds = [1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 1];
        end
      end
      if (r.hand_left > 0)
        l_hand_names = {'left_finger_1_joint_1',
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
        obj.l_hand_joint_names_cache = cell(11, 1);
        for i=1:length(l_hand_names)
          obj.l_hand_joint_names_cache(i) = java.lang.String(l_hand_names{i});
        end
        % not sure what the last two correspond to. for now I'm going to 
        % zero them.
        if (strcmp(r.hand_left_kind, 'robotiq'))
          obj.l_hand_joint_inds = [1, 2, 3, 11, 12, 13, 21, 22, 23, 10, 20];
        elseif (strcmp(r.hand_left_kind, 'robotiq_tendons'))
          obj.l_hand_joint_inds = [1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 1];
        elseif (strcmp(r.hand_left_kind, 'robotiq_simple'))
          obj.l_hand_joint_inds = [1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 1];
        end
      end

      % figure out joint reordering -- atlas_state_t assumes joints
      % in a particular order :P
      switch r.atlas_version
        case 3
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
        case 4
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
                                      'l_arm_shz',
                                      'l_arm_shx',
                                      'l_arm_ely',
                                      'l_arm_elx',
                                      'l_arm_uwy',
                                      'l_arm_mwx',
                                      'r_arm_shz',
                                      'r_arm_shx',
                                      'r_arm_ely',
                                      'r_arm_elx',
                                      'r_arm_uwy',
                                      'r_arm_mwx'
                                      };
        case 5
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
                                      'l_arm_shz',
                                      'l_arm_shx',
                                      'l_arm_ely',
                                      'l_arm_elx',
                                      'l_arm_uwy',
                                      'l_arm_mwx',
                                      'l_arm_lwy',
                                      'r_arm_shz',
                                      'r_arm_shx',
                                      'r_arm_ely',
                                      'r_arm_elx',
                                      'r_arm_uwy',
                                      'r_arm_mwx',
                                      'r_arm_lwy'
                                      };
      end
      obj.reordering = zeros(length(desired_joint_name_order), 1);
      for i=1:length(desired_joint_name_order)
        obj.reordering(i) = find(strcmp(desired_joint_name_order{i}, atlascoordnames));
      end
      
      obj.r = r;
      obj.r_control = r_control;
      obj.l_foot_id = obj.r_control.findLinkId('l_foot');
      obj.r_foot_id = obj.r_control.findLinkId('r_foot');
      obj.nq_control = obj.r_control.getNumPositions();
      if (isa(input_frame, 'MultiCoordinateFrame'))
        obj.frame_nums.atlas_state = input_frame.getFrameNumByNameRecursive('drcFrames.AtlasState');
        obj.frame_nums.right_hand_state = input_frame.getFrameNumByNameRecursive('right_atlasFrames.HandState');
        obj.frame_nums.left_hand_state = input_frame.getFrameNumByNameRecursive('left_atlasFrames.HandState');
        obj.frame_nums.hokuyo_state = input_frame.getFrameNumByNameRecursive('hokuyo');
        obj.frame_nums.left_foot_ft_state = input_frame.getFrameNumByNameRecursive('l_foot_ftForceTorque');
        obj.frame_nums.right_foot_ft_state = input_frame.getFrameNumByNameRecursive('r_foot_ftForceTorque');
      else
        obj.frame_nums.atlas_state = 1;
        obj.frame_nums.right_hand_state = '';
        obj.frame_nums.left_hand_state = '';
        obj.frame_nums.hokuyo_state = '';
        obj.frame_nums.left_foot_ft_state = '';
        obj.frame_nums.right_foot_ft_state = '';
      end
    end
    
    function varargout=mimoOutput(obj,t,~,varargin)
      atlas_state = [];
      right_hand_state = [];
      left_hand_state = [];
      laser_state = [];
      left_ankle_ft_state = zeros(6, 1);
      right_ankle_ft_state = zeros(6, 1);
      
      % Extract particular frame data
      num = obj.frame_nums.atlas_state;
      if isempty(num)
        error(['No atlas state found as input for LCMBroadcastBlock!']);
      elseif length(num)==1
        atlas_state = varargin{num};
      elseif length(num)==2
        atlas_state = varargin{num(1)};
        combined_frame = obj.getInputFrame.getFrameByNum(num(1));
        atlas_state = atlas_state(combined_frame.frame_id==num(2));
      else
        error(['I need to write a general case']);
      end

      num = obj.frame_nums.right_hand_state;
      if (length(num)==1)
        hand_state_tmp = varargin{num};
      elseif (length(num)==2)
        hand_state_tmp = varargin{num(1)};
        combined_frame = obj.getInputFrame.getFrameByNum(num(1));
        hand_state_tmp = hand_state_tmp(combined_frame.frame_id==num(2));
      elseif (length(num) > 2)
        error(['I need to write a general case']);
      end
      if (~isempty(num))
        right_hand_state = [hand_state_tmp(obj.r_hand_joint_inds);
                      hand_state_tmp(length(hand_state_tmp)/2+obj.r_hand_joint_inds)];
        right_hand_state(10:11) = [0;0];
      end

      num = obj.frame_nums.left_hand_state;
      if (length(num)==1)
        hand_state_tmp = varargin{num};
      elseif (length(num)==2)
        hand_state_tmp = varargin{num(1)};
        combined_frame = obj.getInputFrame.getFrameByNum(num(1));
        hand_state_tmp = hand_state_tmp(combined_frame.frame_id==num(2));
      elseif (length(num)>2)
        error(['I need to write a general case']);
      end
      if (~isempty(num))
        left_hand_state = [hand_state_tmp(obj.l_hand_joint_inds);
                      hand_state_tmp(length(hand_state_tmp)/2+obj.l_hand_joint_inds)];
        left_hand_state(10:11) = [0;0];
      end

      num = obj.frame_nums.hokuyo_state;
      laser_state = [];
      if (length(num)>1)
        error(['This shouldnt happen']);
      elseif (length(num)==1)
        laser_state = varargin{num};
      end

      if (~isempty(laser_state))
        laser_spindle_angle = laser_state(1);
        laser_ranges = laser_state(2:end);
      else
        laser_spindle_angle = 0;
        laser_ranges = [];
      end
      
      % Generate forces in ankle
      if (obj.frame_nums.left_foot_ft_state)
        left_ankle_ft_state = varargin{obj.frame_nums.left_foot_ft_state};
        right_ankle_ft_state = varargin{obj.frame_nums.right_foot_ft_state};
      else
        % Get binary foot contact, call it force:
        x = atlas_state;
        fc = obj.r_control.getFootContacts(x(1:obj.nq_control));
        % Scale foot up for the foot that the com is more over
        com = obj.r_control.getCOM(atlas_state(1:obj.r_control.getNumPositions));
        kinsol = doKinematics(obj.r_control,atlas_state(1:obj.r_control.getNumPositions));
        lfootpos = obj.r_control.forwardKin(kinsol,obj.l_foot_id,[0,0,0].');
        rfootpos = obj.r_control.forwardKin(kinsol,obj.r_foot_id,[0,0,0].');
        ldist = lfootpos(1:2)-com(1:2); ldist = ldist.'*ldist;
        rdist = rfootpos(1:2)-com(1:2); rdist = rdist.'*rdist;
        interdist = lfootpos(1:2)-rfootpos(1:2); interdist = interdist.'*interdist;
        left_ankle_ft_state(3) = fc(2)*(800 + 500*rdist/interdist);
        right_ankle_ft_state(3) = fc(1)*(800 + 500*ldist/interdist);
      end
      
      % See if we just passed our publish-timestep for 
      % the foot contact state message, publish if so (and if
      % we're publishing ground truth anyway)
      if (mod(t, obj.fc_publish_period)  < obj.r.timestep)
        foot_contact_est = drc.foot_contact_estimate_t();
        foot_contact_est.utime = t*1000*1000;
        foot_contact_est.left_contact = left_ankle_ft_state(3) > 500;
        foot_contact_est.right_contact = right_ankle_ft_state(3) > 500;
        foot_contact_est.detection_method = 0;
        obj.lc.publish('FOOT_CONTACT_ESTIMATE', foot_contact_est);
      end
      
      % What needs to go out:
      num_dofs = length([atlas_state; right_hand_state; left_hand_state]) / 2;
      atlas_dofs = length(atlas_state)/2;
      right_hand_dofs = length(right_hand_state)/2;
      left_hand_dofs = length(left_hand_state)/2;
      % Robot state publishing (channel depends on whether we're publishing
      % truth (on EST_ROBOT_STATE) or not (ATLAS_STATE)
      if (~obj.publish_truth)
        state_msg = drc.atlas_state_t();
      else
        state_msg = bot_core.robot_state_t();
      end
      
      state_msg.utime = t*1000*1000;
      
      if (obj.publish_truth)
        % ATLAS_STATE has no global pose, but EST_ROBOT_STATE does
        state_msg.pose = bot_core.position_3d_t();
        state_msg.pose.translation = bot_core.vector_3d_t();
        state_msg.pose.rotation = bot_core.quaternion_t();
        state_msg.pose.translation.x = atlas_state(1);
        state_msg.pose.translation.y = atlas_state(2);
        state_msg.pose.translation.z = atlas_state(3);

        yaw = atlas_state(6);
        yaw = mod(yaw, 2*pi);
        if (yaw > pi)
          yaw = yaw - 2*pi;
        end
        q = rpy2quat([atlas_state(4) atlas_state(5) yaw]);
        state_msg.pose.rotation.w = q(1);
        state_msg.pose.rotation.x = q(2);
        state_msg.pose.rotation.y = q(3);
        state_msg.pose.rotation.z = q(4);

        state_msg.twist = bot_core.twist_t();
        state_msg.twist.linear_velocity = bot_core.vector_3d_t();
        state_msg.twist.angular_velocity = bot_core.vector_3d_t();
        state_msg.twist.linear_velocity.x = atlas_state(atlas_dofs+1);
        state_msg.twist.linear_velocity.y = atlas_state(atlas_dofs+2);
        state_msg.twist.linear_velocity.z = atlas_state(atlas_dofs+3);
        avel = rpydot2angularvel(atlas_state(4:6), atlas_state(atlas_dofs+4:atlas_dofs+6));
        state_msg.twist.angular_velocity.x = avel(1);
        state_msg.twist.angular_velocity.y = avel(2);
        state_msg.twist.angular_velocity.z = avel(3);
      end
      
      if (obj.publish_truth)
        % will publish the multisense angle as part of total
        % est_robot_state
        state_msg.num_joints = num_dofs-6+1;
        % only est_robot_state atlas_state message has joint names
        state_msg.joint_name = [obj.joint_names_cache; obj.r_hand_joint_names_cache; obj.l_hand_joint_names_cache; 'hokuyo_joint'];
      else
        state_msg.num_joints = atlas_dofs-6;
      end
      
      state_msg.joint_position=zeros(1,state_msg.num_joints);
      state_msg.joint_velocity=zeros(1,state_msg.num_joints);
      state_msg.joint_effort=zeros(1,state_msg.num_joints);
        
      atlas_pos = atlas_state(7:atlas_dofs);
      atlas_vel = atlas_state(atlas_dofs+7:end);
      if (~obj.publish_truth)
        atlas_pos = atlas_pos(obj.reordering);
        atlas_vel = atlas_vel(obj.reordering);
        state_msg.joint_position =atlas_pos;
        state_msg.joint_velocity = atlas_vel;
      else
        state_msg.joint_position = [atlas_pos; right_hand_state(1:right_hand_dofs); left_hand_state(1:left_hand_dofs); laser_spindle_angle];
        state_msg.joint_velocity = [atlas_vel; right_hand_state(right_hand_dofs+1:end); left_hand_state(left_hand_dofs+1:end); 0];
      end
      state_msg.force_torque = bot_core.force_torque_t();

      % pack in any foot contact info that we have
      state_msg.force_torque.l_foot_force_z = left_ankle_ft_state(3);
      state_msg.force_torque.l_foot_torque_x = left_ankle_ft_state(4);
      state_msg.force_torque.l_foot_torque_y = left_ankle_ft_state(5);
      
      state_msg.force_torque.r_foot_force_z = right_ankle_ft_state(3);
      state_msg.force_torque.r_foot_torque_x = right_ankle_ft_state(4);
      state_msg.force_torque.r_foot_torque_y = right_ankle_ft_state(5);

      
      if (~obj.publish_truth)
        obj.lc.publish('ATLAS_STATE', state_msg);
      else
        obj.lc.publish('EST_ROBOT_STATE', state_msg);
      end
      
      % force an update of the body pose too
      % which needs to have velocity transformed into the 
      % body frame
      pose_body_frame = bot_core.pose_t();
      pose_body_frame.utime = t*1000*1000;
      pose_body_frame.pos = [atlas_state(1), atlas_state(2), atlas_state(3)];
      pose_body_frame.vel = [atlas_state(1+atlas_dofs), atlas_state(2+atlas_dofs), atlas_state(3+atlas_dofs)];
      for i=1:3
        atlas_state(3+i) = mod(atlas_state(3+i), 2*pi);
        if (atlas_state(3+i) > pi)
          atlas_state(3+i) = atlas_state(3+i) - 2*pi;
        end
      end
      q = rpy2quat([atlas_state(4) atlas_state(5) atlas_state(6)]);
      pose_body_frame.vel = quatRotateVec(quatConjugate(q), pose_body_frame.vel);
      pose_body_frame.orientation = [q(1) q(2) q(3) q(4)]; % Rotation
      avel = rpydot2angularvel(atlas_state(4:6), atlas_state(atlas_dofs+4:atlas_dofs+6));
      pose_body_frame.rotation_rate = avel;
      pose_body_frame.rotation_rate = quatRotateVec(quatConjugate(q), pose_body_frame.rotation_rate);
      pose_body_frame.accel = [0 0 0];
      if (obj.publish_truth)
        obj.lc.publish('POSE_BODY', pose_body_frame);
      else
        obj.lc.publish('POSE_BDI', pose_body_frame);
      end
      
      % state_sync expects separate message for the hand state
      robotiq_state = bot_core.joint_state_t();
      if (~obj.publish_truth && ~isempty(right_hand_state))
        robotiq_state.utime = t*1000*1000;
        robotiq_state.num_joints = 11;
        robotiq_state.joint_name = obj.r_hand_joint_names_cache;
        robotiq_state.joint_position = right_hand_state(1:right_hand_dofs);
        robotiq_state.joint_velocity = right_hand_state(right_hand_dofs+1:end);
        robotiq_state.joint_effort = zeros(11, 1);
        obj.lc.publish('ROBOTIQ_RIGHT_STATE', robotiq_state);
      end
      if (~obj.publish_truth && ~isempty(left_hand_state))
        robotiq_state.utime = t*1000*1000;
        robotiq_state.num_joints = 11;
        robotiq_state.joint_name = obj.l_hand_joint_names_cache;
        robotiq_state.joint_position = left_hand_state(1:left_hand_dofs);
        robotiq_state.joint_velocity = left_hand_state(left_hand_dofs+1:end);
        robotiq_state.joint_effort = zeros(11, 1);
        obj.lc.publish('ROBOTIQ_LEFT_STATE', robotiq_state);
      end

      % Send over
      % -- "MULTISENSE_STATE" -- lcm_state msg for joint hokuyo_joint,
      %     labeled with tiem of scan
      % -- also "PRE_SPINDLE_TO_POST_SPINDLE" with the approp transform
      % -- To channel "SCAN", publish populated lcm_laser_msg
      if (~isempty(laser_state))
        % MULTISENSE_STATE and PRE_SPINDLE_TO_POST_SPINDLE, beginning of scan
        multisense_state = bot_core.joint_state_t();
        multisense_state.joint_name = {'hokuyo_joint'};
        
        multisense_state.joint_position = [mod(laser_spindle_angle+pi, 2*pi)];
        multisense_state.joint_velocity = [0.0];
        multisense_state.joint_effort = [0.0];
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
      
      % construct and publish a reasonable microstrain imu message
      % given our states over time
      if (obj.publish_imu)
        this_state = [atlas_state(1:6); atlas_state(atlas_dofs+1:atlas_dofs+6)];
        last_state = obj.last_floating_state.getData();
        obj.last_floating_state.setData(this_state);
        
        % Gyro 
        gyro = this_state(10:12); %rpydot2angularvel(this_state(4:6),this_state(10:12));
        
        % Get acc by time difference velocities
        dt = obj.getSampleTime;
        acc = (this_state(7:9) - last_state(7:9))/dt(1);
        acc = acc + [0;0;9.80665];
        
        % Get them in local frame of pelvis plus BODY_TO_IMU rotation
        quat_world_to_imu = rpy2quat(this_state(4:6) + [0;180;-45]*pi/180);
        %quat_world_to_body = rpy2quat(-this_state(4:6));
        gyro = quatRotateVec(quat_world_to_imu, gyro);
        acc = quatRotateVec(quat_world_to_imu, acc);
        
        % This isn't perfectly accurate as the imu is offset from the
        % very core of the body frame by a small amount (BODY_TO_IMU
        % has nonzero translation).
        imu_msg = bot_core.ins_t();
        imu_msg.utime = t*1000*1000;
        imu_msg.device_time = t*1000*1000;
        imu_msg.gyro = gyro;
        imu_msg.mag = zeros(3, 1);
        imu_msg.accel = acc;
        imu_msg.quat = [0;0;0;0]; %quat_world_to_imu; % not used? 
        imu_msg.pressure = 0;
        imu_msg.rel_alt = 0;
        obj.lc.publish('MICROSTRAIN_INS', imu_msg);
      end
      
      varargout = varargin;
      
    end
  end
  
end
