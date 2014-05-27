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

    function plan = slice(obj, idx)
      plan = obj;
      plan.footsteps = obj.footsteps(idx);
      plan.region_order = obj.region_order(idx);
    end
    
    function plan = extend(obj, final_length, n)
      % Extend a footstep plan by replicating its final n footsteps. Useful for
      % generating seeds for later optimization.
      % @param final_length desired number of footsteps in the extended plan
      % @option n how many final steps to consider (the last n steps will be
      %          repeatedly appended to the footstep plan until the final
      %          length is achieved). Optional. Default: 2
      % @retval plan the extended plan
      if nargin < 3
        n = 2;
      end
      if n > length(obj.footsteps)
        error('DRC:FootstepPlan:NotEnoughStepsToExtend', 'Not enough steps in the plan to extend in the requested manner');
      end
      if final_length <= length(obj.footsteps)
        plan = plan.slice(1:final_length);
      else
        plan = obj;
        j = 1;
        source_ndx = (length(obj.footsteps) - n) + (1:n);
        for k = (length(obj.footsteps) + 1):final_length
          plan.footsteps(k) = plan.footsteps(source_ndx(j));
          plan.region_order(k) = plan.region_order(source_ndx(j));
          plan.footsteps(k).id = plan.footsteps(k-1).id + 1;
          j = mod(j, length(source_ndx)) + 1;
        end
      end
    end
    
    function steps = step_matrix(obj, frame_name)
      % Return the footstep plan poses as a [6 x nsteps] matrix
      if nargin < 2
        frame_name = 'center';
      end
      steps = zeros(6, length(obj.footsteps));
      for j = 1:length(obj.footsteps)
        steps(:,j) = obj.footsteps(j).pos.inFrame(obj.footsteps(j).frames.(frame_name)).double();
      end
    end

    function ts = compute_step_timing(obj)
      % Compute the approximate step timing based on the distance each swing foot must travel.
      % @retval ts a vector of times (in seconds) corresponding to the completion
      %            (return to double support) of each step in the plan. The first
      %            two entries of ts will always be zero, since the first two steps
      %            in the plan correspond to the current locations of the feet.
      ts = zeros(1, length(obj.footsteps));
      for j = 3:length(obj.footsteps)
        [swing_ts, ~, ~, ~] = planSwing(obj.footsteps(j-2), obj.footsteps(j));
        ts(j) = ts(j-1) + swing_ts(end);
      end
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
    function plan = from_footstep_plan_t(msg, biped)
      footsteps = Footstep.empty();
      for j = 1:msg.num_steps
        footsteps(j) = Footstep.from_footstep_t(msg.footsteps(j), biped);
      end
      plan = FootstepPlan(footsteps, msg.params, [], []);
    end

    function plan = blank_plan(biped, nsteps, ordered_body_idx, params, safe_regions)
      footsteps = Footstep.empty();
      
      frames = struct('orig', CoordinateFrame('orig', 6, 'o', {'x', 'y', 'z', 'roll', 'pitch', 'yaw'}),...
                            'center', CoordinateFrame('center', 6, 'c', {'x', 'y', 'z', 'roll', 'pitch', 'yaw'}));
      offset = biped.foot_contact_offsets.right.center;
      frames.orig.addTransform(FootstepContactTransform(frames.orig, ...
                                                            frames.center,...
                                                            offset));
      frames.center.addTransform(FootstepContactTransform(frames.center, ...
                                                            frames.orig,...
                                                            -offset));
      for j = 1:nsteps
        pos = nan(6,1);
        id = j;
        body_idx = ordered_body_idx(mod(j-1, length(ordered_body_idx)) + 1);
        is_in_contact = true;
        pos_fixed = zeros(6,1);
        terrain_pts = [];
        infeasibility = nan;
        walking_params = [];
        footsteps(j) = Footstep(biped, pos, id, body_idx, is_in_contact, pos_fixed, terrain_pts, infeasibility, walking_params, frames);
      end
      region_order = nan(1, nsteps);
      plan = FootstepPlan(footsteps, params, safe_regions, region_order);
    end
  end
end
