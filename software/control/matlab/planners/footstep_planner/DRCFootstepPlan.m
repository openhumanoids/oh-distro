classdef DRCFootstepPlan < FootstepPlan
% Wrapper class for the Drake FootstepPlan object which handles LCM (de)serialization

  properties
    lc = lcm.lcm.LCM.getSingleton();
  end

  methods
    function obj = DRCFootstepPlan(varargin)
      obj = obj@FootstepPlan(varargin{:});
    end

    function msg = to_footstep_plan_t(obj)
      msg = drc.footstep_plan_t();
      msg.num_steps = length(obj.footsteps);
      step_msgs = javaArray('drc.footstep_t', msg.num_steps);
      for j = 1:msg.num_steps
        step_msgs(j) = obj.footsteps(j).to_footstep_t(obj.biped);
      end
      msg.footsteps = step_msgs;
      msg.params = populateLCMFields(drc.footstep_plan_params_t(), obj.params);

      msg.num_iris_regions = length(obj.safe_regions);
      if ~isempty(obj.safe_regions)
        region_msgs = javaArray('drc.iris_region_t', msg.num_iris_regions);
        for j = 1:msg.num_iris_regions
          region_msgs(j) = IRISRegion.to_iris_region_t(obj.safe_regions(j));
        end
        msg.iris_regions = region_msgs;
      end

      if isempty(obj.region_order)
        region_order = -ones(1, length(obj.footsteps));
      else
        region_order = obj.region_order;
        region_order(isnan(region_order)) = -1;
      end
      msg.iris_region_assignments = region_order;

      % Publish a simple position sequence corresponding to the footsteps in this plan (e.g. for debugging state estimation)
      position_msgs = javaArray('bot_core.position_3d_t', msg.num_steps);
      for j = 1:msg.num_steps
        position_msgs(j) = msg.footsteps(j).pos;
      end
      debug_msg = drc.position_3d_sequence_t();
      debug_msg.num_positions = msg.num_steps;
      debug_msg.positions = position_msgs;
      obj.lc.publish('DEBUG_FOOTSTEP_POSES', debug_msg);
    end

    function msg = toLCM(obj)
      msg = obj.to_footstep_plan_t();
    end
  end

  methods(Static)
    function obj = from_footstep_plan_t(msg, biped)
      footsteps = Footstep.empty();
      for j = 1:msg.num_steps
        footsteps(j) = Footstep.from_footstep_t(msg.footsteps(j), biped);
      end

      iris_regions = IRISRegion.empty();
      for j = 1:msg.num_iris_regions
        iris_regions(j) = IRISRegion.from_iris_region_t(msg.iris_regions(j));
      end

      region_order = msg.iris_region_assignments;
      region_order(region_order == -1) = nan;

      obj = DRCFootstepPlan(footsteps, biped, msg.params, iris_regions, region_order);
    end

    function obj = from_drake_footstep_plan(plan)
      obj = DRCFootstepPlan(plan.footsteps, plan.biped, plan.params, plan.safe_regions, plan.region_order);
    end
  end
end
