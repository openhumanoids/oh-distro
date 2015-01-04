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
    foot_indices;
    
    % FC publish period
    fc_publish_period = 0.01;
    
    % Atlas, for usefulness
    r;
    r_control;
    nq;

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
      
      if (isa(obj.getInputFrame, 'AtlasState'))
        atlascoordnames = obj.getInputFrame.getCoordinateNames;
      else
        atlascoordnames = obj.getInputFrame.getFrameByName('AtlasState').getCoordinateNames;
      end
      atlascoordnames = atlascoordnames(7:length(atlascoordnames)/2); % cut off the floating base

      obj.joint_names_cache = cell(length(atlascoordnames), 1);
      % (this is the part that we really don't want to run repeatedly...)
      for i=1:length(atlascoordnames)
        obj.joint_names_cache(i) = java.lang.String(atlascoordnames(i));
      end
      % write down names for the hands. these differ from simul a bit
      % (we have the fourbar explicitely), so I'm doing this by hand...
      if (r.hands>0)
        hand_names = {'right_finger_1_joint_1',
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
        for i=1:length(hand_names)
          obj.r_hand_joint_names_cache(i) = java.lang.String(hand_names{i});
        end
        % not sure what the last two correspond to. for now I'm going to 
        % zero them.
        obj.r_hand_joint_inds = [1, 2, 3, 11, 12, 13, 21, 22, 23, 10, 20];
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
                                      'l_arm_usz',
                                      'l_arm_shx',
                                      'l_arm_ely',
                                      'l_arm_elx',
                                      'l_arm_uwy',
                                      'l_arm_mwx',
                                      'r_arm_usz',
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
                                      'l_arm_usz',
                                      'l_arm_shx',
                                      'l_arm_ely',
                                      'l_arm_elx',
                                      'l_arm_uwy',
                                      'l_arm_mwx',
                                      'l_arm_uwy2',
                                      'r_arm_usz',
                                      'r_arm_shx',
                                      'r_arm_ely',
                                      'r_arm_elx',
                                      'r_arm_uwy',
                                      'r_arm_mwx',
                                      'r_arm_uwy2'
                                      };
      end
      obj.reordering = zeros(length(desired_joint_name_order), 1);
      for i=1:length(desired_joint_name_order)
        obj.reordering(i) = find(strcmp(desired_joint_name_order{i}, atlascoordnames));
      end
      obj.foot_indices = [r.findLinkId('l_foot'), r.findLinkId('r_foot')];
      
      obj.r = r;
      obj.nq = obj.r.getNumPositions();
      obj.r_control = r_control;
      obj.frame_nums.atlas_state = input_frame.getFrameNumByName('AtlasState');
      obj.frame_nums.hand_state = input_frame.getFrameNumByName('HandState');
      obj.frame_nums.hokuyo_state = input_frame.getFrameNumByName('hokuyo');
    end
    
    function varargout=mimoOutput(obj,t,~,varargin)
      atlas_state = [];
      hand_state = [];
      laser_state = [];
      if (length(varargin) == 1)
        % Only atlas state coming in, no need for lots of logic to extract
        % frames
        atlas_state = varargin{1};
      else
        num = obj.frame_nums.atlas_state;
        if length(num)~=1
          error(['No atlas state found as input for LCMBroadcastBlock!']);
        end
        atlas_state = varargin{num};

        num = obj.frame_nums.hand_state;

        if (length(num)>1)
          error(['Ambiguous hand state. No support for two hands yet...']);
        elseif (length(num)==1)
          hand_state_ours = varargin{num};
          % Map it to the hand state the rest of the system understands
          hand_state = [hand_state_ours(obj.r_hand_joint_inds);
                        hand_state_ours(length(hand_state_ours)/2+obj.r_hand_joint_inds)];
          hand_state(10:11) = [0;0];
        end
        
        num = obj.frame_nums.hokuyo_state;
        laser_state = [];
        if (length(num)>1)
          error(['Ambiguous hand state. No support for two hands yet...']);
        elseif (length(num)==1)
          laser_state = varargin{num};
        end
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
        fc = obj.getFootContacts(x(1:obj.nq));
       
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
        state_msg.joint_name = [obj.joint_names_cache; obj.r_hand_joint_names_cache; 'hokuyo_joint'];
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
        % need another factor of pi/2though I wouldn't mind it being earlier rather than later to get drawn multisense to agree with
        % the "true" (functionally true, anyway) mirror position
        state_msg.joint_position = [atlas_pos; hand_state(1:hand_dofs); laser_spindle_angle+pi/2];
        state_msg.joint_velocity = [atlas_vel; hand_state(hand_dofs+1:end); 0];
      end
      state_msg.force_torque = drc.force_torque_t();
      % if we're publishing to satisfy state est we need force sensing from feet
      if (~obj.publish_truth)
        % Get binary foot contact, call it force:
        x = atlas_state;
        fc = obj.getFootContacts(x(1:obj.nq));

        % pack it up
        state_msg.force_torque.l_foot_force_z = fc(1)*1000;
        state_msg.force_torque.r_foot_force_z = fc(2)*1000;
      end
      
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
      
      % state_sync expects separate message for the hand state
      if (~obj.publish_truth && ~isempty(hand_state))
        robotiq_right_state = drc.hand_state_t();
        robotiq_right_state.utime = t*1000*1000;
        robotiq_right_state.num_joints = 11;
        robotiq_right_state.joint_name = obj.r_hand_joint_names_cache;
        robotiq_right_state.joint_position = hand_state(1:hand_dofs);
        robotiq_right_state.joint_velocity = hand_state(hand_dofs+1:end);
        robotiq_right_state.joint_effort = zeros(11, 1);
        obj.lc.publish('ROBOTIQ_RIGHT_STATE', robotiq_right_state);
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
      
      % construct and publish a reasonable microstrain imu message
      % given our states over time
      if (obj.publish_imu)
        this_state = [atlas_state(1:6); atlas_state(atlas_dofs+1:atlas_dofs+6)];
        last_state = obj.last_floating_state.getData();
        obj.last_floating_state.setData(this_state);
        
        % Gyro 
        gyro = rpydot2angularvel(this_state(4:6),this_state(10:12));
        
        % Get acc by time difference velocities
        dt = obj.getSampleTime;
        acc = (this_state(7:9) - last_state(7:9))/dt(1);
        acc = acc + [0;0;9.80665];
        
        % Get them in local frame of pelvis plus BODY_TO_IMU rotation
        quat_body_to_world = rpy2quat(this_state(4:6) + [0; 180; -45+180]*pi/180.);
        quat_world_to_body = quatConjugate(quat_body_to_world);
        gyro = quatRotateVec(quat_body_to_world, gyro);
        acc = quatRotateVec(quat_body_to_world, acc);
        
        % This isn't perfectly accurate as the imu is offset from the
        % very core of the body frame by a small amount (BODY_TO_IMU
        % has nonzero translation).
        imu_msg = microstrain.ins_t();
        imu_msg.utime = t*1000*1000;
        imu_msg.device_time = t*1000*1000;
        imu_msg.gyro = gyro;
        imu_msg.mag = zeros(3, 1);
        imu_msg.accel = acc;
        imu_msg.quat = quat_body_to_world;
        imu_msg.pressure = 0;
        imu_msg.rel_alt = 0;
        obj.lc.publish('MICROSTRAIN_INS', imu_msg);
      end
      
      varargout = varargin;
      
    end
    
    function fc = getFootContacts(obj, q)
      [phiC,~,~,~,idxA,idxB] = obj.r_control.collisionDetect(q,false);
      within_thresh = phiC < 0.002;
      contact_pairs = [idxA(within_thresh); idxB(within_thresh)];

      % The following would be faster but would require us to have
      % hightmaps in Bullet
      %[~,~,idxA,idxB] = obj.r_control.allCollisions(x(1:obj.nq));
      %contact_pairs = [idxA; idxB];

      fc = any(bsxfun(@eq, contact_pairs(:), obj.foot_indices),1)';
    end
  end
  
end
