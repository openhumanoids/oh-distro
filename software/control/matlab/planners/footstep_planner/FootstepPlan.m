classdef FootstepPlan
  properties
    footsteps
  end

  methods
    function obj = FootstepPlan(footsteps)
      obj.footsteps = footsteps;
    end

    function msg = to_footstep_plan_t(obj)
      msg = drc.footstep_plan_t();
      msg.num_steps = length(obj.footsteps);
      step_msgs = javaArray('drc.footstep_t', msg.num_steps);
      for j = 1:msg.num_steps
        step_msgs(j) = obj.footsteps(j).to_footstep_t();
      end
      msg.footsteps = step_msgs;
    end

    function msg = toLCM(obj)
      msg = obj.to_footstep_plan_t();
    end
  end

  methods(Static=true)
    function plan = from_collocation_results(X)
      footsteps = Footstep.empty();
      for j = 1:length(X)
        pos = X(j).pos;
        id = j;
        is_right_foot = X(j).is_right_foot;
        is_in_contact = true;
        pos_fixed = zeros(6,1);
        terrain_pts = [];
        infeasibility = nan;
        walking_params = [];
        footsteps(j) = Footstep(pos, id, is_right_foot, is_in_contact, pos_fixed, terrain_pts, infeasibility, walking_params);
      end
      plan = FootstepPlan(footsteps);
    end

    function plan = from_footstep_plan_t(msg)
      plan = Footstep.empty();
      for j = 1:msg.num_steps
        plan(j) = Footstep.from_footstep_t(msg.footsteps(j));
      end
    end
  end
end
