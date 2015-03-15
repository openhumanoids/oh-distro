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

      obj.plan_monitors{end+1} = drake.util.MessageMonitor(drc.qp_locomotion_plan_t, 'utime');
      obj.plan_channels{end+1} = 'WALKING_CONTROLLER_PLAN_RESPONSE';
      obj.plan_handlers{end+1} = @obj.handle_locomotion_plan;

      obj.plan_monitors{end+1} = drake.util.MessageMonitor(drc.qp_locomotion_plan_t, 'utime');
      obj.plan_channels{end+1} = 'CONFIGURATION_TRAJ';
      obj.plan_handlers{end+1} = @obj.handle_locomotion_plan;

      obj.plan_monitors{end+1} = drake.util.MessageMonitor(drc.utime_t(), 'utime');
      obj.plan_channels{end+1} = 'START_MIT_STAND';
      obj.plan_handlers{end+1} = @obj.handle_stand_default;

      obj.plan_monitors{end+1} = drake.util.MessageMonitor(drc.atlas_behavior_command_t, 'utime');
      obj.plan_channels{end+1} = 'ATLAS_BEHAVIOR_COMMAND';
      obj.plan_handlers{end+1} = @obj.handle_atlas_behavior_command;

     % obj = addLCMTransition(obj,'ATLAS_BEHAVIOR_COMMAND',drc.atlas_behavior_command_t(),'init');
     % obj = addLCMTransition(obj,'CONFIGURATION_TRAJ

    end
    
    function new_plan = handle_stand_default(obj, msg)
      disp('Got a default stand plan')
      msg = drc.utime_t(msg);

      [x0, ~] = obj.atlas_state_coder.decode(drc.robot_state_t(obj.atlas_state_monitor.getMessage()));
      new_plan = DRCQPLocomotionPlan.from_standing_state(x0, obj.robot);

    end

    function new_plan = handle_locomotion_plan(obj, msg)
      disp('Got a locomotion plan')
      msg = drc.qp_locomotion_plan_t(msg);

      new_plan = DRCQPLocomotionPlan.from_qp_locomotion_plan_t(msg, obj.robot);
      
    end

    function new_plan = handle_atlas_behavior_command(obj, msg)
      msg = drc.atlas_behavior_command_t(msg);
      if strcmp(char(msg.command), 'stop') || strcmp(char(msg.command), 'freeze')
        disp('Got an atlas behavior command...going into silent mode');
        new_plan = SilentPlan(obj.robot);
      else
        new_plan = [];
      end
    end

    function new_plan = smoothPlanTransition(obj, t, x, new_plan)
      % Make the transition to the new plan as smooth as possible
      new_plan.start_time = t;

      % Use the previously commanded state to override the first (or only) knot point in the new trajectory. This is necessary to prevent the robot from suddenly drooping or jerking at the start of a plan, since the initial state of the plan may not match the robot's current desired state. 
      if ~isempty(obj.data.last_qp_input) && isfield(new_plan, 'qtraj')
        if isnumeric(new_plan.qtraj)
          new_plan.qtraj(7:end) = obj.data.last_qp_input.whole_body_data.q_des(7:end);
        else
          if isa(new_plan.qtraj, 'PPTrajectory')
            % The qtraj is a piecewise polynomial, so we'll just add a constant offset to its first segment to ensure that its initial qdes is the same as our previously published qdes.
            [ts, coefs, l, k, d] = unmkpp(new_plan.qtraj.pp);
            coefs = reshape(coefs, [d, l, k]);
            q0 = fasteval(new_plan.qtraj, ts(1));
            delta_q = obj.data.last_qp_input.whole_body_data.q_des - q0;
            coefs(7:end,1,end) = coefs(7:end,1,end) + delta_q(7:end);
            if d > 1
              new_plan.qtraj = PPTrajectory(pchipDeriv(ts, [coefs(:,:,end), fasteval(new_plan.qtraj, ts(end))], coefs(:,:,end-1)));
            else
              new_plan.qtraj = PPTrajectory(pchip(ts, coefs(:,:,end)));
            end
          elseif isa(new_plan.qtraj, 'ConstantTrajectory')
            q0 = double(new_plan.qtraj.pt);
            delta_q = obj.data.last_qp_input.whole_body_data.q_des - q0;
            q_smooth = [q0(1:6); q0(7:end) + delta_q(7:end)];
            new_plan.qtraj = ConstantTrajectory(q_smooth);
          else
            warning('Got a new plan whose qtraj is neither a constant nor a PPTrajectory. I can not smooth the transition to this plan.');
          end
        end
      end
    end

    function updateQueueLCM(obj, t, x)
      % check for new plans, and use them to modify the queue
      for j = 1:length(obj.plan_monitors)
        msg = obj.plan_monitors{j}.getNextMessage(0);
        if isempty(msg)
          continue
        end
        try
          % try to handle it using supplied handler
          disp('handling new plan on channel:')
          obj.plan_channels{j}
          new_plan = obj.plan_handlers{j}(msg);
          if ~isempty(new_plan)
            new_plan = obj.smoothPlanTransition(t, x, new_plan);
            obj.switchToPlan(new_plan);
            break;
          end
        catch e
          report = e.getReport();
          disp(report)
        end
      end
    end

    function run(obj)
      % subscribe to everything we're listening to -- plan channels + robot state
      obj.lc.subscribe(obj.atlas_state_channel, obj.atlas_state_monitor);
      for j = 1:length(obj.plan_monitors)
        obj.lc.subscribe(obj.plan_channels{j}, obj.plan_monitors{j});
      end

      % state_data = obj.atlas_state_monitor.getNextMessage(5);
      % [x, t] = obj.atlas_state_coder.decode(drc.robot_state_t(state_data));
      obj.data.plan_queue = {};
      
      disp('DRCPlanEval now listening');

      while 1
        % check for new state
        state_data = obj.atlas_state_monitor.getNextMessage(0);
        if isempty(state_data)
          continue
        end
        [x, t] = obj.atlas_state_coder.decode(drc.robot_state_t(state_data));
        obj.updateQueueLCM(t, x);
        if ~isempty(obj.data.plan_queue)
          qp_input = obj.getQPControllerInput(t, x);
          if ~isempty(qp_input)
            qp_input.param_set_name = [qp_input.param_set_name, '_', obj.mode]; % send _sim or _hardware param variant
            encodeQPInputLCMMex(qp_input);
            obj.data.last_qp_input = qp_input;
          end
        end
      end
    end
  end
  
end
