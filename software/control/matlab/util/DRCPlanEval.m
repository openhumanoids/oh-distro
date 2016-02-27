classdef DRCPlanEval < bipedControllers.BipedPlanEval
  % A standalone PlanEval (DRC-specific) extension 
  % that knows how to read plans from LCM, and offers
  % a blocking run() method to receive and publish
  % plans in a tight loop.

  properties  
    lc
    mode = 'sim';

    msg_monitors = {};
    msg_constructors = {};
    msg_handlers = {};
    msg_channels = {};
    msg_timeouts = {};
    msg_names = {};

    atlas_state_coder;

    pause_state = 0;
    recovery_state = 0;
    recovery_enabled = 1;
    bracing_enabled = 1;
    reactive_recovery_planner;
    bracing_plan;
    contact_force_detected;
    last_status_msg_time;

    last_plan_msg_utime = 0;
  end


  properties(Constant)
   PAUSE_NONE = 0;
   PAUSE_NOW = 1;
   STOP_WALKING_ASAP = 2;

   RECOVERY_NONE = 0;
   RECOVERY_ACTIVE = 1;
   RECOVERY_BRACING = 2;

 end

  methods
    function obj = DRCPlanEval(r, mode, varargin)
      typecheck(r,'DRCAtlas');
      obj = obj@bipedControllers.BipedPlanEval(r, varargin{:});
      obj.contact_force_detected = zeros(obj.robot.getNumBodies(), 1);
      assert(strcmp(obj.mode, 'sim') || strcmp(obj.mode, 'hardware'), 'bad mode: %s', mode);
      obj.mode = mode;

      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.atlas_state_coder = r.getStateFrame().lcmcoder;
      obj.reactive_recovery_planner = QPReactiveRecoveryPlan(r);
      obj.bracing_plan = BracingPlan(obj.robot);

      obj = obj.addLCMInterface('foot_contact', 'FOOT_CONTACT_ESTIMATE', @drc.foot_contact_estimate_t, 0, @obj.handle_foot_contact);
      obj = obj.addLCMInterface('walking_plan', 'WALKING_CONTROLLER_PLAN_RESPONSE', @drc.qp_locomotion_plan_t, 0, @obj.handle_locomotion_plan);
      obj = obj.addLCMInterface('manip_plan', 'CONFIGURATION_TRAJ', @drc.qp_locomotion_plan_t, 0, @obj.handle_locomotion_plan);
      obj = obj.addLCMInterface('start_stand', 'START_MIT_STAND', @bot_core.utime_t, 0, @obj.handle_stand_default);
      obj = obj.addLCMInterface('atlas_behavior', 'ATLAS_BEHAVIOR_COMMAND', @drc.behavior_command_t, 0, @obj.handle_drc_behavior_command);
      obj = obj.addLCMInterface('pause_manip', 'COMMITTED_PLAN_PAUSE', @drc.plan_control_t, 0, @obj.handle_pause);
      obj = obj.addLCMInterface('stop_walking', 'STOP_WALKING', @drc.plan_control_t, 0, @obj.handle_pause);
      obj = obj.addLCMInterface('state', 'EST_ROBOT_STATE', @bot_core.robot_state_t, -1, @obj.handle_state);
      obj = obj.addLCMInterface('recovery_trigger', 'RECOVERY_TRIGGER', @drc.recovery_trigger_t, 0, @obj.handle_recovery_trigger);
      obj = obj.addLCMInterface('recovery_enable', 'RECOVERY_ENABLE', @drc.boolean_t, 0, @obj.handle_recovery_enable);
      obj = obj.addLCMInterface('bracing_enable', 'BRACING_ENABLE', @drc.boolean_t, 0, @obj.handle_bracing_enable);
      obj = obj.addLCMInterface('bracing_plan', 'BRACE_FOR_FALL', @drc.recovery_trigger_t, 0, @obj.handle_bracing_plan);
    end

    function obj = addLCMInterface(obj, name, channel, msg_constructor, timeout, handler)
      obj.msg_names{end+1} = name;
      obj.msg_constructors{end+1} = msg_constructor;
      obj.msg_handlers{end+1} = handler;
      obj.msg_timeouts{end+1} = timeout;
      obj.msg_channels{end+1} = channel;
      obj.msg_monitors{end+1} = drake.util.MessageMonitor(msg_constructor(), 'utime');
    end

    function obj = setupSubscriptions(obj)
      % subscribe to everything we're listening to -- plan channels + robot state
      for j = 1:length(obj.msg_channels)
        obj.lc.subscribe(obj.msg_channels{j}, obj.msg_monitors{j});
      end
    end

    function LCMReceive(obj)
      for j = 1:length(obj.msg_channels)
        msg_data = obj.msg_monitors{j}.getNextMessage(0);
        if ~isempty(msg_data)
          msg = obj.msg_constructors{j}(msg_data);
          obj.msg_handlers{j}(msg);
        end
      end
    end

    function handle_state(obj, msg)
      [x, t] = obj.atlas_state_coder.decode(msg);
      obj.t = t;
      obj.x = x;
    end

    function handle_foot_contact(obj, msg)
      obj.contact_force_detected(obj.robot.foot_body_id.right) = double(msg.right_contact > 0.5);
      obj.contact_force_detected(obj.robot.foot_body_id.left) = double(msg.left_contact > 0.5);
    end
    
    function handle_stand_default(obj, msg)
      disp('Got a default stand plan')
      obj.last_plan_msg_utime = msg.utime;
      new_plan = QPLocomotionPlanCPPWrapper(QPLocomotionPlanSettings.fromStandingState(obj.x, obj.robot));
      obj.switchToPlan(new_plan);
    end

    function handle_locomotion_plan(obj, msg)
      disp('Got a locomotion plan')
      obj.recovery_state = obj.RECOVERY_NONE;
      obj.last_plan_msg_utime = msg.utime;
      new_plan = DRCQPLocomotionPlan.from_qp_locomotion_plan_t(msg, obj.robot);
      obj.switchToPlan(QPLocomotionPlanCPPWrapper(obj.smoothPlanTransition(new_plan)));
    end

    function handle_bracing_plan(obj, msg)
      if (msg.activate)
        if (obj.bracing_enabled || msg.override)
          if ~isempty(obj.x)
            if (obj.recovery_state ~= obj.RECOVERY_BRACING)
              disp('Acting on a bracing plan')
              obj.recovery_state = obj.RECOVERY_BRACING;
              obj.switchToPlan(obj.bracing_plan);
            end
          end
        end
      end
    end
    
    function handle_drc_behavior_command(obj, msg)
      cmd = char(msg.command);
      if ~(strcmp(cmd, 'user') || strcmp(cmd, 'USER'))
        disp('Got an atlas behavior command...going into silent mode');
        obj.recovery_state = obj.RECOVERY_NONE;
        obj.last_plan_msg_utime = msg.utime;
        obj.switchToPlan(SilentPlan(obj.robot));
      end
    end

    function handle_pause(obj, msg)
      if msg.control == msg.PAUSE
        disp('Got a plan pause NOW');
        obj.pause_state = obj.PAUSE_NOW;
      elseif msg.control == msg.TERMINATE
        disp('Got a stop walking ASAP');
        obj.pause_state = obj.STOP_WALKING_ASAP;
      else
        % handle this somehow
        disp('I want to resume but I dont know how yet');
      end
    end

    function handle_recovery_trigger(obj, msg)
      if msg.activate
        if obj.recovery_enabled || msg.override
          if obj.recovery_state == obj.RECOVERY_NONE && ~isempty(obj.x)
            disp('Entering reactive recovery mode!');
            obj.last_plan_msg_utime = msg.utime;
            obj.reactive_recovery_planner.resetInitialization();
            disp('initialization reset');
            obj.switchToPlan(obj.reactive_recovery_planner);
            disp('switched to plan');
            obj.recovery_state = obj.RECOVERY_ACTIVE;
            disp('recovery active');
          end
        end
      else
        if obj.recovery_state == obj.RECOVERY_ACTIVE
          disp('Exiting reactive recovery mode!');
          obj.recovery_state = obj.RECOVERY_NONE;
          new_plan = QPLocomotionPlanCPPWrapper(QPLocomotionPlanSettings.fromStandingState(obj.x, obj.robot));
          obj.switchToPlan(new_plan);
        end
      end
    end

    function handle_bracing_enable(obj, msg)
      obj.bracing_enabled = msg.data;
      obj.sendStatus()
    end

    function handle_recovery_enable(obj, msg)
      obj.recovery_enabled = msg.data;
      obj.sendStatus()
    end

    function new_plan = smoothPlanTransition(obj, new_plan)
      % Make the transition to the new plan as smooth as possible

      % Use the previously commanded state to override the first (or only) knot point in the new trajectory. This is necessary to prevent the robot from suddenly drooping or jerking at the start of a plan, since the initial state of the plan may not match the robot's current desired state. 
      if ~isempty(obj.qp_input)
        if isnumeric(new_plan.qtraj)
          new_plan.qtraj(1:end) = obj.qp_input.whole_body_data.q_des(1:end);
        else
          if isa(new_plan.qtraj, 'PPTrajectory')
            % The qtraj is a piecewise polynomial, so we'll just add a constant offset to its first segment to ensure that its initial qdes is the same as our previously published qdes.
            [ts, coefs, l, k, d] = unmkpp(new_plan.qtraj.pp);
            coefs = reshape(coefs, [d, l, k]);
            q0 = fasteval(new_plan.qtraj, ts(1));
            delta_q = obj.qp_input.whole_body_data.q_des - q0;
            coefs(1:end,1,end) = coefs(1:end,1,end) + delta_q(1:end);
            if k > 1
              new_plan.qtraj = PPTrajectory(pchipDeriv(ts, [coefs(:,:,end), fasteval(new_plan.qtraj, ts(end))], coefs(:,:,end-1)));
            else
              new_plan.qtraj = PPTrajectory(pchip(ts, [coefs(:,:,end) coefs(:,end,end)]));
            end
          elseif isa(new_plan.qtraj, 'ConstantTrajectory')
            q_smooth = obj.qp_input.whole_body_data.q_des;
            new_plan.qtraj = q_smooth;
          else
            warning('Got a new plan whose qtraj is neither a constant nor a PPTrajectory. I can not smooth the transition to this plan.');
          end
        end
      end
    end

    function pauseIfRequested(obj)
      if obj.pause_state == obj.STOP_WALKING_ASAP && ~isempty(obj.qp_input)
        have_right_foot = obj.contact_force_detected(obj.robot.foot_body_id.right);
        have_left_foot = obj.contact_force_detected(obj.robot.foot_body_id.left);
        if have_right_foot && have_left_foot
          disp('Got double support, pausing now.')
          obj.pause_state = obj.PAUSE_NONE;
          obj.switchToPlan(QPLocomotionPlanCPPWrapper(QPLocomotionPlanSettings.fromStandingState(obj.x, obj.robot)));
        end
      elseif obj.pause_state == obj.PAUSE_NOW
        disp('freezing now')
        obj.switchToPlan(FrozenPlan(obj.qp_input));
        obj.pause_state = obj.PAUSE_NONE;
      end
    end

    function obj = switchToPlan(obj, new_plan)
      obj = switchToPlan@bipedControllers.BipedPlanEval(obj, new_plan);
      obj.sendStatus();
    end

    function current_plan = getCurrentPlan(obj, t, x)
      while true
        current_plan = obj.plan_queue{1};
        if ~current_plan.isFinished(t, x);
          break
        end
        disp('current plan is finished')
        if length(obj.plan_queue) == 1
          obj.plan_queue{1} = current_plan.getSuccessor(t, x);
        else
          obj.plan_queue(1) = [];
        end
        obj.sendStatus();
      end
    end

    function sendStatus(obj)
      if isempty(obj.last_status_msg_time) || (obj.t - obj.last_status_msg_time) > 0.2 || obj.last_status_msg_time > obj.t
        if ~isempty(obj.plan_queue)
          current_plan = obj.plan_queue{1};
          if isa(current_plan, 'QPLocomotionPlanCPPWrapper')
            if strcmp(current_plan.settings.gain_set, 'standing')
              execution_flag = drc.plan_status_t.EXECUTION_STATUS_FINISHED;
              plan_type = drc.plan_status_t.STANDING;
            elseif strcmp(current_plan.settings.gain_set, 'walking')
              execution_flag = drc.plan_status_t.EXECUTION_STATUS_EXECUTING;
              plan_type = drc.plan_status_t.WALKING;
            elseif strcmp(current_plan.settings.gain_set, 'manip')
              execution_flag = drc.plan_status_t.EXECUTION_STATUS_EXECUTING;
              plan_type = drc.plan_status_t.MANIPULATING;
            else
              execution_flag = drc.plan_status_t.EXECUTION_STATUS_EXECUTING;
              plan_type = drc.plan_status_t.UNKNOWN;
            end
          elseif isa(current_plan, 'QPReactiveRecoveryPlan')
            execution_flag = drc.plan_status_t.EXECUTION_STATUS_EXECUTING;
            plan_type = drc.plan_status_t.RECOVERING;
          elseif isa(current_plan, 'FrozenPlan')
            execution_flag = drc.plan_status_t.EXECUTION_STATUS_FINISHED;
            plan_type = drc.plan_status_t.STANDING;
          elseif isa(current_plan, 'BracingPlan')
            execution_flag = drc.plan_status_t.EXECUTION_STATUS_EXECUTING;
            plan_type = drc.plan_status_t.BRACING;
          else
            execution_flag = drc.plan_status_t.EXECUTION_STATUS_NO_PLAN;
            plan_type = drc.plan_status_t.DUMMY;
          end
        else
          execution_flag = drc.plan_status_t.EXECUTION_STATUS_NO_PLAN;
          plan_type = drc.plan_status_t.DUMMY;
          current_plan = [];
        end
        obj.last_status_msg_time = obj.t;
        status_msg = drc.controller_status_t();
        status_msg.utime = obj.t * 1e6;
        status_msg.state = plan_type;
        obj.lc.publish('CONTROLLER_STATUS', status_msg);

        plan_status_msg = drc.plan_status_t();
        plan_status_msg.utime = obj.t * 1e6;
        plan_status_msg.plan_type = plan_type;
        plan_status_msg.execution_status = execution_flag;
        plan_status_msg.last_plan_msg_utime = obj.last_plan_msg_utime;
        if isempty(current_plan)
          plan_status_msg.last_plan_start_utime = 0;
        else
          plan_status_msg.last_plan_start_utime = current_plan.start_time * 1e6;
        end
        plan_status_msg.recovery_enabled = obj.recovery_enabled;
        plan_status_msg.bracing_enabled = obj.bracing_enabled;
        obj.lc.publish('PLAN_EXECUTION_STATUS', plan_status_msg);

      end
    end

    function run(obj)
      obj = obj.setupSubscriptions();

      obj.plan_queue = {};
      
      disp('DRCPlanEval now listening');

      while 1
        try
          obj.LCMReceive();
          obj.pauseIfRequested();
          if ~isempty(obj.plan_queue)
            qp_input = obj.getQPControllerInput(obj.t, obj.x, obj.contact_force_detected);

            if ~isempty(qp_input)
              if isnumeric(qp_input)
                qp_input = drake.lcmt_qp_controller_input(qp_input);
              end
              obj.lc.publish('QP_CONTROLLER_INPUT', qp_input);
              obj.qp_input = qp_input;
            end
          end
          obj.sendStatus();
        catch e
          disp('error in planEval loop:')
          disp(e.getReport())
        end
      end
    end
  end
  
end
