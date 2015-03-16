classdef DRCPlanEval < atlasControllers.AtlasPlanEval
  % A standalone PlanEval (DRC-specific) extension 
  % that knows how to read plans from LCM, and offers
  % a blocking run() method to receive and publish
  % plans in a tight loop.

  properties  
    lc
    mode = 'sim';
    lcm_interfaces = struct('name', {}, 'channel', {}, 'msg_constructor', {}, 'timeout', {}, 'handler', {}, 'monitor', {});
    atlas_state_coder;
  end

  methods
    function obj = DRCPlanEval(r, mode, varargin)
      typecheck(r,'DRCAtlas');
      obj = obj@atlasControllers.AtlasPlanEval(r, varargin{:});
      obj.data = DRCPlanEvalData(struct('plan_queue', {obj.data.plan_queue}));
      obj.data.contact_force_detected = zeros(obj.robot_property_cache.num_bodies, 1);
      assert(strcmp(obj.mode, 'sim') || strcmp(obj.mode, 'hardware'), 'bad mode: %s', mode);
      obj.mode = mode;

      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.atlas_state_coder = r.getStateFrame().lcmcoder;
      obj = obj.addLCMInterface('state', 'EST_ROBOT_STATE', @drc.robot_state_t, -1, @obj.handle_state);
      obj = obj.addLCMInterface('foot_contact', 'FOOT_CONTACT_ESTIMATE', @drc.foot_contact_estimate_t, 0, @obj.handle_foot_contact);
      obj = obj.addLCMInterface('walking_plan', 'WALKING_CONTROLLER_PLAN_RESPONSE', @drc.qp_locomotion_plan_t, 0, @obj.handle_locomotion_plan);
      obj = obj.addLCMInterface('manip_plan', 'CONFIGURATION_TRAJ', @drc.qp_locomotion_plan_t, 0, @obj.handle_locomotion_plan);
      obj = obj.addLCMInterface('start_stand', 'START_MIT_STAND', @drc.utime_t, 0, @obj.handle_stand_default);
      obj = obj.addLCMInterface('atlas_behavior', 'ATLAS_BEHAVIOR_COMMAND', @drc.atlas_behavior_command_t, 0, @obj.handle_atlas_behavior_command);
      obj = obj.addLCMInterface('pause_manip', 'COMMITTED_PLAN_PAUSE', @drc.plan_control_t, 0, @obj.handle_pause);
      obj = obj.addLCMInterface('stop_walking', 'STOP_WALKING', @drc.plan_control_t, 0, @obj.handle_pause);
    end

    function obj = addLCMInterface(obj, name, channel, msg_constructor, timeout, handler)
      obj.lcm_interfaces(end+1) = struct('name', name,...
                                         'channel', channel,...
                                         'msg_constructor', msg_constructor,... 
                                         'timeout', timeout,...
                                         'handler', handler,...
                                         'monitor', drake.util.MessageMonitor(msg_constructor(), 'utime'));
    end

    function obj = setupSubscriptions(obj)
      % subscribe to everything we're listening to -- plan channels + robot state
      for j = 1:length(obj.lcm_interfaces)
        obj.lc.subscribe(obj.lcm_interfaces(j).channel, obj.lcm_interfaces(j).monitor);
      end
    end

    function LCMReceive(obj)
      for j = 1:length(obj.lcm_interfaces)
        msg_data = obj.lcm_interfaces(j).monitor.getNextMessage(obj.lcm_interfaces(j).timeout);
        if ~isempty(msg_data)
          msg = obj.lcm_interfaces(j).msg_constructor(msg_data);
          obj.lcm_interfaces(j).handler(msg);
        end
      end
    end

    function handle_state(obj, msg)
      [x, t] = obj.atlas_state_coder.decode(msg);
      obj.data.t = t;
      obj.data.x = x;
    end

    function handle_foot_contact(obj, msg)
      obj.data.contact_force_detected(obj.robot.foot_body_id.right) = double(msg.right_contact > 0.5);
      obj.data.contact_force_detected(obj.robot.foot_body_id.left) = double(msg.left_contact > 0.5);
    end
    
    function handle_stand_default(obj, msg)
      disp('Got a default stand plan')
      new_plan = DRCQPLocomotionPlan.from_standing_state(obj.data.x, obj.robot);
      obj.switchToPlan(obj.smoothPlanTransition(new_plan));
    end

    function handle_locomotion_plan(obj, msg)
      disp('Got a locomotion plan')
      new_plan = DRCQPLocomotionPlan.from_qp_locomotion_plan_t(msg, obj.robot);
      obj.switchToPlan(obj.smoothPlanTransition(new_plan));
    end

    function handle_atlas_behavior_command(obj, msg)
      if strcmp(char(msg.command), 'stop') || strcmp(char(msg.command), 'freeze')
        disp('Got an atlas behavior command...going into silent mode');
        obj.switchToPlan(SilentPlan(obj.robot));
      end
    end

    function handle_pause(obj, msg)
      disp('Got a plan pause ASAP:')
      if (msg.control == msg.PAUSE || msg.control == msg.TERMINATE)
        obj.data.pause_state = obj.data.PAUSE_ASAP;
        % maybe terminate is different? it's what sent for
        % stop walking right now.
      else
        % handle this somehow
        disp('I want to resume but I dont know how yet');
      end
    end

    function new_plan = smoothPlanTransition(obj, new_plan)
      % Make the transition to the new plan as smooth as possible
      new_plan.start_time = obj.data.t;

      % Use the previously commanded state to override the first (or only) knot point in the new trajectory. This is necessary to prevent the robot from suddenly drooping or jerking at the start of a plan, since the initial state of the plan may not match the robot's current desired state. 
      if ~isempty(obj.data.qp_input) && isfield(new_plan, 'qtraj')
        if isnumeric(new_plan.qtraj)
          new_plan.qtraj(7:end) = obj.data.qp_input.whole_body_data.q_des(7:end);
        else
          if isa(new_plan.qtraj, 'PPTrajectory')
            % The qtraj is a piecewise polynomial, so we'll just add a constant offset to its first segment to ensure that its initial qdes is the same as our previously published qdes.
            [ts, coefs, l, k, d] = unmkpp(new_plan.qtraj.pp);
            coefs = reshape(coefs, [d, l, k]);
            q0 = fasteval(new_plan.qtraj, ts(1));
            delta_q = obj.data.qp_input.whole_body_data.q_des - q0;
            coefs(7:end,1,end) = coefs(7:end,1,end) + delta_q(7:end);
            if d > 1
              new_plan.qtraj = PPTrajectory(pchipDeriv(ts, [coefs(:,:,end), fasteval(new_plan.qtraj, ts(end))], coefs(:,:,end-1)));
            else
              new_plan.qtraj = PPTrajectory(pchip(ts, coefs(:,:,end)));
            end
          elseif isa(new_plan.qtraj, 'ConstantTrajectory')
            q0 = double(new_plan.qtraj.pt);
            delta_q = obj.data.qp_input.whole_body_data.q_des - q0;
            q_smooth = [q0(1:6); q0(7:end) + delta_q(7:end)];
            new_plan.qtraj = ConstantTrajectory(q_smooth);
          else
            warning('Got a new plan whose qtraj is neither a constant nor a PPTrajectory. I can not smooth the transition to this plan.');
          end
        end
      end
    end

    function pauseIfRequested(obj)
      % TODO: should this use sensed foot contact instead of planned contact?
      if obj.data.pause_state == obj.data.PAUSE_ASAP && ~isempty(obj.data.qp_input)
        have_right_foot = 0;
        have_left_foot = 0;
        for i=1:length(obj.data.qp_input.support_data)
          % If this support is not forbidden
          if (any(obj.data.qp_input.support_data(i).support_logic_map))
            % I'm sure this can be vectorized better
            if (obj.data.qp_input.support_data(i).body_id == obj.robot.foot_body_id.right)
              have_right_foot = 1;
            elseif (obj.data.qp_input.support_data(i).body_id == obj.robot.foot_body_id.left)
              have_left_foot = 1;
            end
          end
        end
        if have_right_foot && have_left_foot
          disp('Got double support, pausing now.')
          obj.data.pause_state = obj.data.PAUSE_NONE;
          obj.switchToPlan(DRCQPLocomotionPlan.from_standing_state(obj.data.x, obj.robot));
        end
      end
    end

    function run(obj)
      obj = obj.setupSubscriptions();

      obj.data.plan_queue = {};
      
      disp('DRCPlanEval now listening');

      while 1
        try
          obj.LCMReceive();
          obj.pauseIfRequested();
          if ~isempty(obj.data.plan_queue)
            qp_input = obj.getQPControllerInput(obj.data.t, obj.data.x, obj.data.contact_force_detected);
            if ~isempty(qp_input)
              qp_input.param_set_name = [qp_input.param_set_name, '_', obj.mode]; % send _sim or _hardware param variant
              encodeQPInputLCMMex(qp_input);
              obj.data.qp_input = qp_input;
            end
          end
        catch e
          disp('error in planEval loop:')
          disp(e.getReport())
        end
      end
    end
  end
  
end
