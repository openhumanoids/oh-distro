classdef DRCPlanEval < atlasControllers.AtlasPlanEval
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
    contact_force_detected;
    last_status_msg_time;
  end


  properties(Constant)
   PAUSE_NONE = 0;
   PAUSE_NOW = 1;
   STOP_WALKING_ASAP = 2;
 end

  methods
    function obj = DRCPlanEval(r, mode, varargin)
      typecheck(r,'DRCAtlas');
      obj = obj@atlasControllers.AtlasPlanEval(r, varargin{:});
      obj.contact_force_detected = zeros(obj.robot_property_cache.num_bodies, 1);
      assert(strcmp(obj.mode, 'sim') || strcmp(obj.mode, 'hardware'), 'bad mode: %s', mode);
      obj.mode = mode;

      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.atlas_state_coder = r.getStateFrame().lcmcoder;
      obj = obj.addLCMInterface('foot_contact', 'FOOT_CONTACT_ESTIMATE', @drc.foot_contact_estimate_t, 0, @obj.handle_foot_contact);
      obj = obj.addLCMInterface('walking_plan', 'WALKING_CONTROLLER_PLAN_RESPONSE', @drc.qp_locomotion_plan_t, 0, @obj.handle_locomotion_plan);
      obj = obj.addLCMInterface('manip_plan', 'CONFIGURATION_TRAJ', @drc.qp_locomotion_plan_t, 0, @obj.handle_locomotion_plan);
      obj = obj.addLCMInterface('start_stand', 'START_MIT_STAND', @drc.utime_t, 0, @obj.handle_stand_default);
      obj = obj.addLCMInterface('atlas_behavior', 'ATLAS_BEHAVIOR_COMMAND', @drc.atlas_behavior_command_t, 0, @obj.handle_atlas_behavior_command);
      obj = obj.addLCMInterface('pause_manip', 'COMMITTED_PLAN_PAUSE', @drc.plan_control_t, 0, @obj.handle_pause);
      obj = obj.addLCMInterface('stop_walking', 'STOP_WALKING', @drc.plan_control_t, 0, @obj.handle_pause);
      obj = obj.addLCMInterface('state', 'EST_ROBOT_STATE', @drc.robot_state_t, -1, @obj.handle_state);
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
      new_plan = DRCQPLocomotionPlan.from_standing_state(obj.x, obj.robot);
      obj.switchToPlan(new_plan);
    end

    function handle_locomotion_plan(obj, msg)
      disp('Got a locomotion plan')
      new_plan = DRCQPLocomotionPlan.from_qp_locomotion_plan_t(msg, obj.robot);
      obj.switchToPlan(obj.smoothPlanTransition(new_plan));
      % if isa(new_plan.qtraj, 'Trajectory')
      %   disp('Automatically generating a standing plan from the end of this plan.')
      %   obj.appendPlan(QPLocomotionPlan.from_standing_state(new_plan.qtraj.eval(new_plan.qtraj.tspan(end)),...
      %                                                       obj.robot,...
      %                                                       new_plan.supports(end),...
      %                                                       struct('center_pelvis', false)));
      % end
    end

    function handle_atlas_behavior_command(obj, msg)
      if strcmp(char(msg.command), 'stop') || strcmp(char(msg.command), 'freeze')
        disp('Got an atlas behavior command...going into silent mode');
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

    function new_plan = smoothPlanTransition(obj, new_plan)
      % Make the transition to the new plan as smooth as possible
      new_plan.start_time = [];

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
      % TODO: should this use sensed foot contact instead of planned contact?
      if obj.pause_state == obj.STOP_WALKING_ASAP && ~isempty(obj.qp_input)
        have_right_foot = 0;
        have_left_foot = 0;
        for i=1:length(obj.qp_input.support_data)
          % If this support is not forbidden
          if (any(obj.qp_input.support_data(i).support_logic_map))
            % I'm sure this can be vectorized better
            if (obj.qp_input.support_data(i).body_id == obj.robot.foot_body_id.right)
              have_right_foot = 1;
            elseif (obj.qp_input.support_data(i).body_id == obj.robot.foot_body_id.left)
              have_left_foot = 1;
            end
          end
        end
        if have_right_foot && have_left_foot
          disp('Got double support, pausing now.')
          obj.pause_state = obj.PAUSE_NONE;
          obj.switchToPlan(DRCQPLocomotionPlan.from_standing_state(obj.x, obj.robot));
        end
      elseif obj.pause_state == obj.PAUSE_NOW
        disp('freezing now')
        obj.switchToPlan(FrozenPlan(obj.qp_input));
        obj.pause_state = obj.PAUSE_NONE;
      end
    end

    function sendStatus(obj)
      if isempty(obj.last_status_msg_time) || (obj.t - obj.last_status_msg_time) > 0.2
        if ~isempty(obj.plan_queue)
          current_plan = obj.plan_queue{1};
          if strcmp(current_plan.gain_set, 'standing')
            state_flag = drc.controller_status_t.STANDING;
          elseif strcmp(current_plan.gain_set, 'walking')
            state_flag = drc.controller_status_t.WALKING;
          elseif strcmp(current_plan.gain_set, 'manip')
            state_flag = drc.controller_status_t.MANIPULATING;
          else
            state_flag = drc.controller_status_t.UNKNOWN;
          end
        else
          state_flag = drc.controller_status_t.DUMMY;
        end
        obj.last_status_msg_time = obj.t;
        status_msg = drc.controller_status_t();
        status_msg.utime = obj.t * 1e6;
        status_msg.state = state_flag;
        obj.lc.publish('CONTROLLER_STATUS', status_msg);
      end
    end

    function run(obj)
      obj = obj.setupSubscriptions();

      obj.plan_queue = {};
      
      disp('DRCPlanEval now listening');

      while 1
        try
          % t0 = tic();
          obj.LCMReceive();
          % fprintf(1, 'recieve: %fs\n', toc(t0))
          % t0 = tic();
          obj.pauseIfRequested();
          % fprintf(1, 'pause: %fs\n', toc(t0))
          if ~isempty(obj.plan_queue)
            % t0 = tic()
            qp_input = obj.getQPControllerInput(obj.t, obj.x, obj.contact_force_detected);
            % fprintf(1, 'get input: %fs\n', toc(t0));
            if ~isempty(qp_input)
              if length(qp_input.param_set_name) <= length(obj.mode) + 1 || ~strcmp(qp_input.param_set_name(end-length(obj.mode):end), ['_', obj.mode])
                qp_input.param_set_name = [qp_input.param_set_name, '_', obj.mode]; % send _sim or _hardware param variant
              end
              % t0 = tic();
              encodeQPInputLCMMex(qp_input);
              % fprintf(1, 'encode: %fs\n', toc(t0));
              obj.qp_input = qp_input;
            end
          end
          % t0 = tic();
          obj.sendStatus();
          % fprintf(1, 'send status: %fs\n', toc(t0));
        catch e
          disp('error in planEval loop:')
          disp(e.getReport())
        end
      end
    end
  end
  
end
