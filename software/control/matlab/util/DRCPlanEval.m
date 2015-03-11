classdef DRCPlanEval < atlasControllers.AtlasPlanEval
  % A standalone PlanEval (DRC-specific) extension 
  % that knows how to read plans from LCM, and offers
  % a blocking run() method to receive and publish
  % plans in a tight loop.

  properties  
    lc
    monitor

    atlas_state_monitor;
    atlas_state_channel;

    plan_monitors = {};
    plan_channels = {};
    plan_handlers = {};

  end

  methods
    function obj = DRCPlanEval(r, varargin)
      typecheck(r,'DRCAtlas');
      obj = obj@atlasControllers.AtlasPlanEval(r, varargin{:});

      obj.lc = lcm.lcm.LCM.getSingleton();

      obj.atlas_state_monitor = obj.robot.getStateFrame;
      obj.atlas_state_channel = 'EST_ROBOT_STATE';

      obj.plan_monitors{end+1} = drake.util.MessageMonitor(drc.walking_plan_t, 'utime');
      obj.plan_channels{end+1} = 'WALKING_CONTROLLER_PLAN_RESPONSE';
      obj.plan_handlers{end+1} = @obj.handle_walking_plan;

      obj.plan_monitors{end+1} = drake.util.MessageMonitor(drc.utime_t(), 'utime');
      obj.plan_channels{end+1} = 'START_MIT_STAND';
      obj.plan_handlers{end+1} = @obj.handle_stand_default;

     % ,'stand');
     % obj = addLCMTransition(obj,'ATLAS_BEHAVIOR_COMMAND',drc.atlas_behavior_command_t(),'init');
     % obj = addLCMTransition(obj,'CONFIGURATION_TRAJ

    end
    
    function new_plan = handle_stand_default(obj, msg)
      msg = drc.utime_t(msg);

      S = load(obj.robot.fixed_point_file);
      x0 = double(S.xstar);
      new_plan = StandingPlan.from_standing_state(x0, obj.robot);

      disp('Got a default stand plan')
    end

    function new_plan = handle_walking_plan(obj, msg)
      msg = drc.walking_plan_t(msg);

      % Convert the plan from a walking plan msg to 
      % 
      new_plan = DRCQPWalkingPlan.from_walking_plan_t(msg, obj.robot);
      
      disp('Got a walking plan')
    end

    function run(obj)
      % subscribe to everything we're listening to -- plan channels + robot state
      obj.atlas_state_monitor.subscribe(obj.atlas_state_channel);
      for j = 1:length(obj.plan_monitors)
        obj.lc.subscribe(obj.plan_channels{j}, obj.plan_monitors{j});
      end
      
      % Add a do-nothing plan to queue

      disp('DRCPlanEval now listening');

      while 1
        % check for new state
        [x, t] = obj.atlas_state_monitor.getNextMessage(5);
        if isempty(x)
          continue
        end

        % check for new plans
        for j = 1:length(obj.plan_monitors)
          msg = obj.plan_monitors{j}.getNextMessage(5);
          if isempty(msg)
            continue
          end
          try
            % try to handle it using supplied handler
            new_plan = obj.plan_handlers{j}(msg);
            new_plan.start_time = t;
            obj = obj.switchToPlan(new_plan);
          catch e
            report = e.getReport();
            disp(report)
          end
        end

        % get our current plan and publish if it is legit
        if (~isempty(obj.data.plan_queue))
          qp_input = obj.getQPControllerInput(t, x);
          % and get it out to the controller
          encodeQPInputLCMMex(qp_input);
        end

        %disp('\\O\\ /O/')
      end
    end
  end
  
end
