classdef StatelessFootstepPlanner
  properties
    biped
  end

  methods
    function obj = StatelessFootstepPlanner(biped)
      typecheck(biped, 'Biped');
      obj.biped = biped;
    end

    function plan_footsteps(obj, request)
      x0 = obj.biped.getStateFrame().lcmcoder.decode(request.initial_state);
      q0 = x0(1:end/2);
      foot_orig = obj.biped.feetPosition(q0);

      terrain = obj.biped.getTerrain();
      if ismethod(terrain, 'setMapMode')
        obj.biped.setTerrain(terrain.setMapMode(request.params.map_command));
      end

      goal_pos = StatelessFootstepPlanner.compute_goal_pos(obj.biped, request);

      params_set = {request.params};
      if request.params.leading_foot == drc.footstep_plan_params_t.LEAD_AUTO
        new_params_set = {};
        for p = params_set
          rparams = p{1};
          lparams = p{1};
          rparams.leading_foot = drc.footstep_plan_params_t.LEAD_RIGHT;
          % rparams.right_foot_lead = 1;
          lparams.leading_foot = drc.footstep_plan_params_t.LEAD_LEFT;
          % lparams.right_foot_lead = 0;
          new_params_set{end+1} = rparams;
          new_params_set{end+1} = lparams;
        end
        params_set = new_params_set;
      end
      best_steps = [];
      best_length = inf;
      best_cost = inf;
      for p = params_set
        params = struct(p{1});
        params.right_foot_lead = params.leading_foot; % for backwards compatibility
        if params.planning_mode == drc.footstep_plan_params_t.MODE_AUTO
          X = footstepCollocation(obj.biped, foot_orig, goal_pos, params);
        else
          X = footstepLineSearch(obj.biped, foot_orig, goal_pos.center, params);
        end
        step_vect = encodeCollocationSteps([X(2:end).pos]);
        [steps, steps_rel] = decodeCollocationSteps(step_vect);
        l = length(X);
        c = footstepCostFun(steps, steps_rel, goal_pos, logical(params.right_foot_lead));
        if l < best_length || c < best_cost % always prefer fewer steps
          best_steps = X;
          best_length = l;
          best_cost = c;
        end
      end

      X = best_steps;

      for j = 1:length(X)
        X(j).id = j;
        X(j).params = request.default_step_params;
        if j <= 2
          X(j).pos_fixed = ones(6,1);
        else
          X(j).pos_fixed = zeros(6,1);
        end
        X(j).is_in_contact = true;
      end
    end

  end
  methods (Static=true)
    function goal_pos = compute_goal_pos(biped, request)
      if request.num_goal_steps == 0
        pos = StatelessFootstepPlanner.decodePosition3d(request.goal_pos);
        goal_pos.center = pos;
        goal_pos.right = Biped.stepCenter2FootCenter(pos, true, request.params.nom_step_width);
        goal_pos.left = Biped.stepCenter2FootCenter(pos, false, request.params.nom_step_width);
      else
        for j = 1:(min([2, request.num_goal_steps]))
          pos = StatelessFootstepPlanner.decodePosition3d(request.goal_steps(j).pos);
          if request.goal_steps(j).is_right_foot
            goal_pos.right = biped.footOrig2Contact(pos, 'center', true);
          else
            goal_pos.left = biped.footOrig2Contact(pos, 'center', false);
          end
        end
        if ~isfield(goal_pos, 'right')
          goal_pos.right = Biped.stepCenter2FootCenter(...
                                   Biped.footCenter2StepCenter(goal_pos.left, false, request.params.nom_step_width),...
                                   true, request.params.nom_step_width);
        elseif ~isfield(goal_pos, 'left')
          goal_pos.left = Biped.stepCenter2FootCenter(...
                                   Biped.footCenter2StepCenter(goal_pos.right, true, request.params.nom_step_width),...
                                   false, request.params.nom_step_width);
        end
        goal_pos.center = mean([goal_pos.right, goal_pos.left], 2);
        goal_pos.center(4:6) = goal_pos.right(4:6) + 0.5 * angleDiff(goal_pos.right(4:6), goal_pos.left(4:6));
      end
    end

    function pose = decodePosition3d(position_3d)
      pose = zeros(6,1);
      pose(1:3) = [position_3d.translation.x; position_3d.translation.y; position_3d.translation.z];
      [pose(4), pose(5), pose(6)] = quat2angle([position_3d.rotation.w,...
                                          position_3d.rotation.x,...
                                          position_3d.rotation.y,...
                                          position_3d.rotation.z], 'XYZ');
    end
  end
end


