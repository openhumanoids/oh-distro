classdef FootstepPlan
  properties
    footsteps
    params
    safe_regions
    region_order
  end

  methods
    function obj = FootstepPlan(footsteps, params, safe_regions, region_order)
      obj.footsteps = footsteps;
      obj.params = struct(params);
      obj.safe_regions = safe_regions;
      obj.region_order = region_order;
    end

    function msg = to_footstep_plan_t(obj)
      msg = drc.footstep_plan_t();
      msg.num_steps = length(obj.footsteps);
      step_msgs = javaArray('drc.footstep_t', msg.num_steps);
      for j = 1:msg.num_steps
        step_msgs(j) = obj.footsteps(j).to_footstep_t();
      end
      msg.footsteps = step_msgs;
      msg.params = obj.params;
    end

    function msg = toLCM(obj)
      msg = obj.to_footstep_plan_t();
    end

    function varargout = sanity_check(obj)
      ok = true;
      body_idxs = [obj.footsteps.body_idx];
      if any(body_idxs(1:end-1) == body_idxs(2:end))
        ok = false;
        if nargout < 1
          error('Body indices should not repeat.');
        end
      end
      varargout = {ok};
    end
  end

  methods(Static=true)
    function plan = from_collocation_results(X, params, safe_regions, region_order)
      footsteps = Footstep.empty();
      for j = 1:length(X)
        pos = X(j).pos;
        id = j;
        body_idx = X(j).body_idx;
        is_in_contact = true;
        pos_fixed = zeros(6,1);
        terrain_pts = [];
        infeasibility = nan;
        walking_params = [];
        footsteps(j) = Footstep(pos, id, body_idx, is_in_contact, pos_fixed, terrain_pts, infeasibility, walking_params);
      end
      plan = FootstepPlan(footsteps, params, safe_regions, region_order);
    end

    function plan = from_footstep_plan_t(msg)
      footsteps = Footstep.empty();
      for j = 1:msg.num_steps
        footsteps(j) = Footstep.from_footstep_t(msg.footsteps(j));
      end
      plan = FootstepPlan(footsteps);
    end

    function plan = blank_plan(nsteps, ordered_body_idx, params, safe_regions)
      footsteps = Footstep.empty();
      for j = 1:nsteps
        pos = nan(6,1);
        id = j;
        body_idx = ordered_body_idx(mod(j-1, length(ordered_body_idx)) + 1);
        is_in_contact = true;
        pos_fixed = zeros(6,1);
        terrain_pts = [];
        infeasibility = nan;
        walking_params = [];
        footsteps(j) = Footstep(pos, id, body_idx, is_in_contact, pos_fixed, terrain_pts, infeasibility, walking_params);
      end
      region_order = nan(1, nsteps);
      plan = FootstepPlan(footsteps, params, safe_regions, region_order);
    end
  end
end
