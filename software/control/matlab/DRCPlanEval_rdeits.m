classdef DRCPlanEval < atlasControllers.AtlasPlanEval
  properties
    mode = 'sim';
    lcm_interface;
    lc;
  end

  methods
    function obj = DRCPlanEval(r, mode, varargin)
      obj = obj@atlasControllers.AtlasPlanEval(r, varargin{:});
      assert(strcmp(obj.mode, 'sim') || strcmp(obj.mode, 'hardware'), 'bad mode: %s', mode);
      obj.mode = mode;

      obj.lc = lcm.lcm.LCM.getSingleton();

      obj.lcm_interfaces = struct('monitor', {}, 'channel', {}, 'constructor', {}, 'handler', {});
      obj.lcm_interfaces(end+1).channel = 'WALKING_CONTROLLER_PLAN_RESPONSE';
      obj.lcm_interfaces(end).monitor = drake.util.MessageMonitor(drc.walking_plan_t, 'utime');
      obj.lcm_interfaces(end).constructor = @drc.walking_plan_t;
      obj.lcm_interfaces(end).handler = @obj.handleWalkingMsg;
      obj.lc.subscribe(obj.lcm_interfaces(end).channel, obj.lcm_interfaces(end).monitor);
    end

    function current_plan = getCurrentPlan(obj, t, x)
      obj.handleLCM();
      current_plan = getCurrentPlan@atlasControllers.AtlasPlanEval(obj, t, x);
    end

    function qp_input = getQPControllerInput(obj, t, x)
      qp_input = getQPControllerInput@atlasControllers.AtlasPlanEval(obj, t, x);
      qp_input.param_set_name = [qp_input.param_set_name, '_', obj.mode];
    end

    function handleLCM(obj)
      for j = 1:length(obj.lcm_interfaces)
        new_msg_data = obj.lcm_interfaces(j).monitor.getNextMessage(0);
        if ~isempty(new_msg_data)
          new_msg = obj.lcm_interfaces(j).constructor(new_msg_data);
          obj.lcm_interfaces(j).handler(new_msg);
        end
      end
    end

    function handleWalkingMsg(msg)
      S = mxDeserialize(msg.S);

      if isa(S,'Trajectory')
        S = fasteval(S,0); % S is always constant
      end

      s1 = mxDeserialize(msg.s1);
%       s1dot = mxDeserialize(msg.s1dot);
      s2 = mxDeserialize(msg.s2, 'uint8');
%       s2dot = mxDeserialize(msg.s2dot);
      supports = mxDeserialize(msg.supports);
      if iscell(supports)
        supports = [supports{:}];
      end
      comtraj = mxDeserialize(msg.comtraj);
      zmptraj = mxDeserialize(msg.zmptraj);
      q0 = mxDeserialize(msg.qtraj);
      if ~isnumeric(q0)
        if isa(q0, 'ConstantTrajectory')
          q0 = fasteval(q0, 0);
        else
          error('qtraj must be constant');
        end
      end
      plan = QPWalkingPlan(obj.robot);
      plan.x0 = [q0; zeros(obj.robot.getNumVelocities(), 1)];
      plan.support_times = msg.support_times;
      plan.supports = supports;
      plan.link_constraints = mxDeserialize(msg.link_constraints);
      plan.zmptraj = 

  end
end

