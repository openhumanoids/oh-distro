classdef SitStandWrapper
  
  properties
    robot;
    lc;
    state_monitor;
    plan_options;
    r
    nq
    handle_1;
    handle_2;
    data_file
  end
  
  methods
    
    % atlas version should be 4 or 5
    function obj = SitStandWrapper(atlas_version)
      if nargin < 1
        atlas_version = 4;
      end

      if ~((atlas_version == 4) || (atlas_version == 5))
        error('DRC:SitStandWrapper','atlas_version must be 4 or 5');
      end

      example_options = struct();
      example_options = applyDefaults(example_options, struct('use_mex', true,...
        'use_bullet', false,...
        'navgoal', [1.0;0;0;0;0;0],...
        'quiet', true,...
        'num_steps', 4,...
        'terrain', RigidBodyFlatTerrain));
      
      % silence some warnings
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
      
 

      if atlas_version == 4
        atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v4/model_minimal_contact.urdf'];
        atlas_convex_hull = [getenv('DRC_BASE'),'/software/models/atlas_v4/model_convex_hull.urdf'];
        obj.data_file = [getenv('DRC_BASE'),'/software/control/matlab/planners/chair_standup/chair_standup_data.mat'];
      else
        atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v5/model_minimal_contact.urdf'];
        atlas_convex_hull = [getenv('DRC_BASE'),'/software/models/atlas_v5/model_convex_hull.urdf'];
        obj.data_file = [getenv('DRC_BASE'),'/software/control/matlab/planners/chair_standup/chair_data_v5.mat'];
      end

      obj.plan_options.data_file = obj.data_file;

     % construct robot model
      clear options;
      options.atlas_version = atlas_version;
      options.floating = true;
      options.ignore_self_collisions = true;
      options.ignore_friction = true;
      options.dt = 0.001;
      options.terrain = example_options.terrain;
      options.use_bullet = example_options.use_bullet;
      options.hand_right = 'robotiq_weight_only';
      options.hand_left = 'robotiq_weight_only';      
      r = DRCAtlas(atlas_urdf,options);
      obj.handle_1 = addpathTemporary([getenv('DRC_BASE'),'/software/control/matlab/planners/chair_standup']);
      obj.handle_2 = addpathTemporary([getenv('DRC_BASE'),'/software/control/matlab/planners/prone']);

      clear options_planning;
      options_planning.atlas_version = atlas_version;
      options_planning.floating = true;
      options_planning.hand_right = 'robotiq_weight_only';
      options_planning.hand_left = 'robotiq_weight_only';
      robot = DRCAtlas(atlas_convex_hull,options_planning);
      chair_height = 0.6;
      T = zeros(4,4);
      T(1:3,1:3) = eye(3);
      T(1,4) = 0.1;
      T(4,4) = 1;
      box = RigidBodyBox([1;2;2*chair_height],[0.07;0;0],[0;0;0]);
      r = r.addVisualGeometryToBody(1,box);
      r = compile(r);
      obj.r = r;
      obj.robot = robot;
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.state_monitor = drake.util.MessageMonitor(drc.robot_state_t, 'utime');
      obj.lc.subscribe('EST_ROBOT_STATE', obj.state_monitor);

      obj = obj.setDefaultPlanOptions();
      obj.nq = obj.r.getNumPositions();

    end
    
    function PlanAndPublish(obj,plan_type,speed,execute_flag,chair_height,use_default_initial_pose)
      
      if ~any(strcmp(plan_type,{'sit','stand','squat','stand_from_squat','one_leg_stand','stand_from_one_leg',...
        'lean','hold_with_pelvis_contact','sit_from_current','hold_without_pelvis_contact'}))
        error('DRC:PLANSITSTAND','plan type must be one of {sit,stand,squat,stand_from_squat,one_leg_stand,lean,hold_with_pelvis_contact,sit_from_current}');
      end
      
      if nargin < 3
        speed = 1;
      end
      
      if nargin < 4
        execute_flag = 0;
      end
      
      if nargin < 5
        chair_height = 1/2;
      end
      
      if nargin < 6
        use_default_initial_pose = 0;
      end
      
      if use_default_initial_pose && any(strcmp(plan_type,{'stand','stand_from_squat'}))
        chair_height = 1/2;
      end
      
      r = obj.r;
      robot = obj.robot;
      
      data = [];
      if use_default_initial_pose
        load(obj.data_file);
        if any(strcmp(plan_type,{'sit','squat','one_leg_stand','lean'}))
          q0 = q_sol(:,3);
        elseif strcmp(plan_type,'stand_from_one_leg')
          data = load([getenv('DRC_BASE'),'/software/control/matlab/planners/chair_standup/chair_poses.mat']);
          q0 = data.q_one_foot;
        else
          data = load([getenv('DRC_BASE'),'/software/control/matlab/planners/chair_standup/chair_poses.mat']);
          q0 = data.deep_squat;
        end
        x = [q0;0*q0];
      else
        while isempty(data)
          data = obj.state_monitor.getNextMessage(10);
        end
        [x, t] = r.getStateFrame().lcmcoder.decode(data);
      end
      
      
      
      handle = addpathTemporary([getenv('DRC_BASE'),'/software/control/matlab/planners/chair_standup']);
      
      clear options;
      options = obj.plan_options;
      options.speed = speed;
      options.chair_height = chair_height;

      if obj.plan_options.use_new_planner
        [qtraj,supports,support_times] = PlanSitStand_new.plan(obj.robot,x,plan_type,options);
      else
        [qtraj,supports,support_times] = PlanSitStand.plan(obj.robot,x,plan_type,options);
      end
      

      robot = r.getManipulator();
      handle_2 = addpathTemporary([getenv('DRC_BASE'),'/software/control/matlab/planners/prone']);
      kpt = KinematicPoseTrajectory(robot,{});
      robot = kpt.addVisualContactPoints(robot);
      v = robot.constructVisualizer();
      qtraj = qtraj.setOutputFrame(robot.getPositionFrame());
      v.playback(qtraj,struct('slider',true));

      % handles the publishing of the message
      T = qtraj.getBreaks();
      X = qtraj.eval(T);
      rpp = RobotPlanPublisher('COMMITTED_ROBOT_PLAN_WITH_SUPPORTS',true,r.getStateFrame.coordinates(1:obj.nq));      
      if execute_flag
        disp('do you want to publish this plan?')
        keyboard;
        rpp.publishPlanWithSupports(X,T,supports,support_times);
      end
      
    end

    function obj = useMex(obj,flag)
      obj.use_mex = flag;
    end

    function obj = setDefaultPlanOptions(obj)
      obj.plan_options.use_mex = 1;
      obj.plan_options.pelvis_contact_angle = 0;
      obj.plan_options.use_new_planner = 1;
      obj.plan_options.back_gaze_bound = 0.3;
      obj.plan_options.shrink_factor = 0.6;
      obj.plan_options.back_gaze_bound_tight = 0.01;
      obj.plan_options.sit_back_distance = 0.15;
      obj.plan_options.back_gaze_tight.bound = 0.3;
      obj.plan_options.back_gaze_tight.angle = 0;
      obj.plan_options.pelvis_gaze_bound = 0.05;
      obj.plan_options.pelvis_gaze_angle = 0;
      obj.plan_options.bky_angle = -0.2;
    end


    function computeGravityTorqueConstraint(obj)
      %% Torque Constraint
      robot = obj.robot.getManipulator;
      r = obj.robot;
      joint_names = robot.getPositionFrame.coordinates;
      idx_arm = ~cellfun('isempty',strfind(joint_names,'arm'));
      idx_back = ~cellfun('isempty',strfind(joint_names,'back'));
      % idx = or(idx_arm,idx_back);
      idx = or(idx_arm,idx_back);
      names_arm_back = joint_names(idx);
      torque_multiplier = 0.5;
      torque_multiplier_back = 1;
      pmin = Point(robot.getInputFrame,r.umin);
      pmax = Point(robot.getInputFrame,r.umax);
      
      
      lb = zeros(length(names_arm_back),1);
      ub = lb;
      joint_idx = zeros(length(names_arm_back),1);
      
      
      for j = 1:length(joint_idx)
        name = names_arm_back{j};
        joint_idx(j) = r.findPositionIndices(name);
        lb(j) = pmin.(name)*torque_multiplier;
        ub(j) = pmax.(name)*torque_multiplier;
        if strfind(name,'back')
          lb(j) = pmin.([name])*torque_multiplier_back;
          ub(j) = pmax.([name])*torque_multiplier_back;
        end
        if strfind(name,'back_bky')
          lb(j) = -190;
          ub(j) = 190;
          back_bky_idx = j;
        end
        if strfind(name,'back_bkx')
          lb(j) = -190;
          ub(j) = 190;
          back_bkx_idx = j;
        end
      end
      gct = GravityCompensationTorqueConstraint(robot,joint_idx,lb,ub);

      nq = robot.getNumPositions();

      data = [];
      while isempty(data)
          data = obj.state_monitor.getNextMessage(10);
      end
      [x, t] = obj.r.getStateFrame().lcmcoder.decode(data);

      q0 = x(1:nq);
      kinsol = robot.doKinematics(q0);
      c = gct.eval(0,kinsol);

      disp('back_bky')
      c(back_bky_idx)
      disp('back_bkx')
      c(back_bkx_idx)

    end


  end
end