classdef DRCPlanEval < atlasControllers.AtlasPlanEval
  % A standalone PlanEval (DRC-specific) extension 
  % that knows how to read plans from LCM, and offers
  % a blocking run() method to receive and publish
  % plans in a tight loop.

  properties  
    lc
    mode = 'sim';

    atlas_state_coder;
    atlas_state_monitor;
    atlas_state_channel;

    plan_monitors = {};
    plan_channels = {};
    plan_handlers = {};

  end

  methods
    function obj = DRCPlanEval(r, mode, varargin)
      typecheck(r,'DRCAtlas');
      obj = obj@atlasControllers.AtlasPlanEval(r, varargin{:});
      assert(strcmp(obj.mode, 'sim') || strcmp(obj.mode, 'hardware'), 'bad mode: %s', mode);
      obj.mode = mode;

      obj.lc = lcm.lcm.LCM.getSingleton();

      obj.atlas_state_monitor = drake.util.MessageMonitor(drc.robot_state_t, 'utime');
      obj.atlas_state_coder = r.getStateFrame().lcmcoder;
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
      disp('Got a default stand plan')
      msg = drc.utime_t(msg);

      [x0, ~] = obj.atlas_state_coder.decode(drc.robot_state_t(obj.atlas_state_monitor.getMessage()));
      new_plan = StandingPlan.from_standing_state(x0, obj.robot);

    end

    function new_plan = handle_walking_plan(obj, msg)
      disp('Got a walking plan')
      msg = drc.walking_plan_t(msg);

      % Convert the plan from a walking plan msg to 
      % 
      new_plan = DRCQPWalkingPlan.from_walking_plan_t(msg, obj.robot);
      
    end

    function current_plan = getCurrentPlan(obj, t, x)
      % check for new plans, and use them to modify the queue
      for j = 1:length(obj.plan_monitors)
        msg = obj.plan_monitors{j}.getNextMessage(5);
        if isempty(msg)
          continue
        end
        try
          % try to handle it using supplied handler
          disp('handling new plan on channel:')
          obj.plan_channels{j}
          new_plan = obj.plan_handlers{j}(msg);
          new_plan.start_time = t;
          obj.switchToPlan(new_plan);
          break;
        catch e
          report = e.getReport();
          disp(report)
        end
      end

      % Pass the actual queue handling off to the superclass
      current_plan = getCurrentPlan@atlasControllers.AtlasPlanEval(obj, t, x);
    end

    function run(obj)
      % subscribe to everything we're listening to -- plan channels + robot state
      obj.lc.subscribe(obj.atlas_state_channel, obj.atlas_state_monitor);
      for j = 1:length(obj.plan_monitors)
        obj.lc.subscribe(obj.plan_channels{j}, obj.plan_monitors{j});
      end

      % state_data = obj.atlas_state_monitor.getNextMessage(5);
      % [x, t] = obj.atlas_state_coder.decode(drc.robot_state_t(state_data));
      obj.data.plan_queue = {SilentPlan()};
      
      disp('DRCPlanEval now listening');

      while 1
        % check for new state
        state_data = obj.atlas_state_monitor.getNextMessage(5);
        if isempty(state_data)
          continue
        end
        [x, t] = obj.atlas_state_coder.decode(drc.robot_state_t(state_data));
        qp_input = obj.getQPControllerInput(t, x);
        if ~isempty(qp_input)
          qp_input.param_set_name = [qp_input.param_set_name, '_', obj.mode]; % send _sim or _hardware param variant
          encodeQPInputLCMMex(qp_input);
        end
      end
    end
  end
  
end
