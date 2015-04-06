classdef SitStandWrapper
  
  properties
    robot;
    lc;
    state_monitor;
    plan_options;
    r
    handle_1;
    handle_2;
  end
  
  methods
    
    function obj = SitStandWrapper()
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
      
      % construct robot model
      options.floating = true;
      options.ignore_self_collisions = true;
      options.ignore_friction = true;
      options.dt = 0.001;
      options.terrain = example_options.terrain;
      options.use_bullet = example_options.use_bullet;
      options.hand_right = 'robotiq_weight_only';
      options.hand_left = 'robotiq_weight_only';
      atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v4/model_minimal_contact.urdf'];
      atlas_convex_hull = [getenv('DRC_BASE'),'/software/models/atlas_v4/model_convex_hull.urdf'];
      r = DRCAtlas(atlas_convex_hull,options);
      obj.handle_1 = addpathTemporary([getenv('DRC_BASE'),'/software/control/matlab/planners/chair_standup']);
      obj.handle_2 = addpathTemporary([getenv('DRC_BASE'),'/software/control/matlab/planners/prone']);

      clear options_planning;
      options_planning.floating = true;
      options.hand_right = 'robotiq_weight_only';
      options.hand_left = 'robotiq_weight_only';
      robot = DRCAtlas(atlas_convex_hull,options_planning);
      chair_height = 1/2;
      box = RigidBodyBox([1;2;2*chair_height]);
      r = r.addVisualGeometryToBody(1,box);
      r = compile(r);
      obj.r = r;
      obj.robot = robot;
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.state_monitor = drake.util.MessageMonitor(drc.robot_state_t, 'utime');
      obj.lc.subscribe('EST_ROBOT_STATE', obj.state_monitor);

      obj = obj.setDefaultPlanOptions();

    end
    
    function PlanAndPublish(obj,plan_type,speed,execute_flag,chair_height,use_default_initial_pose)
      
      if ~any(strcmp(plan_type,{'sit','stand','squat','stand_from_squat','one_leg_stand','stand_from_one_leg',...
        'lean','hold_with_pelvis_contact','sit_from_current'}))
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
        load([getenv('DRC_BASE'),'/software/control/matlab/planners/chair_standup/chair_standup_data.mat']);
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
      
      standup_sitdown_plan = QPLocomotionPlan.from_quasistatic_qtraj(r,qtraj,struct('supports',supports,'support_times',support_times));
      msg = DRCQPLocomotionPlan.toLCM(standup_sitdown_plan);
      
      if ~execute_flag
        robot = r.getManipulator();
        handle_2 = addpathTemporary([getenv('DRC_BASE'),'/software/control/matlab/planners/prone']);
        kpt = KinematicPoseTrajectory(robot,{});
        robot = kpt.addVisualContactPoints(robot);
        v = robot.constructVisualizer();
        qtraj = qtraj.setOutputFrame(robot.getPositionFrame());
        v.playback(qtraj,struct('slider',true));
      end
      
      if execute_flag
        disp('do you want to publish this plan?')
        keyboard;
        obj.lc.publish('CONFIGURATION_TRAJ',msg);
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
      obj.plan_options.shrink_factor = 0.5;
    end
  end
end