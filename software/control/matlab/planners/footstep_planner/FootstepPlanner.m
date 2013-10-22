classdef FootstepPlanner < DRCPlanner
  properties
    biped
    monitors
    hmap_ptr
    options
    goal_pos
    adjusted_footsteps
    defaults
    needs_plan
    steps
    old_steps
  end
  
  methods
    function obj = FootstepPlanner(biped)
      typecheck(biped, 'Biped');
      
      obj = obj@DRCPlanner();
      obj.biped = biped;

      obj = addInput(obj,'goal', 'WALKING_GOAL', 'drc.walking_goal_t', false, true, true);
      obj = addInput(obj,'x0','EST_ROBOT_STATE',obj.biped.getStateFrame().lcmcoder,true,true,false);
      obj = addInput(obj, 'plan_con', 'FOOTSTEP_PLAN_CONSTRAINT', drc.footstep_plan_t(), false, true,false);
      obj = addInput(obj, 'plan_commit', 'COMMITTED_FOOTSTEP_PLAN', drc.footstep_plan_t(), false, true,false);
      obj = addInput(obj, 'plan_reject', 'REJECTED_FOOTSTEP_PLAN', drc.footstep_plan_t(), false, true,false);
      obj = addInput(obj, 'step_seq', 'DESIRED_FOOT_STEP_SEQUENCE', drc.traj_opt_constraint_t(), false, true,true);
      obj.goal_pos = [];
      obj.adjusted_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
      obj.defaults = struct('max_num_steps', 10, 'min_num_steps', 0, 'timeout', inf, 'step_height', 0.05, 'step_speed', 1.5, 'nom_step_width', 0.26, 'max_forward_step', 0.5, 'nom_forward_step', 0.15, 'follow_spline', false, 'ignore_terrain', false, 'right_foot_lead', true, 'mu', 1, 'behavior', drc.walking_goal_t.BEHAVIOR_BDI_STEPPING); 
      obj.needs_plan = false;
    end

    function obj = updateGoal(obj, data, changed, changelist)
      if changed
        if isfield(data, 'goal'); info = struct(data.goal); else info = struct(); end
        for x = {'max_num_steps', 'min_num_steps', 'timeout', 'step_height', 'step_speed', 'nom_step_width', 'nom_forward_step', 'max_forward_step','follow_spline', 'ignore_terrain', 'right_foot_lead', 'mu', 'behavior'}
          if isfield(info, x{1}) && ~isnan(info.(x{1}))
            if ~isfield(obj.options, x{1}) || obj.options.(x{1}) ~= info.(x{1});
              obj.options.(x{1}) = info.(x{1});
              if ~isempty(obj.goal_pos); obj.needs_plan = true; end
              if strcmp(x{1}, 'right_foot_lead')
                obj.adjusted_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
              end
            end
          elseif isfield(obj.defaults, x{1})
            obj.options.(x{1}) = obj.defaults.(x{1});
          elseif isfield(obj.biped, x{1})
            obj.options.(x{1}) = obj.biped.(x{1});
          end
        end
        if (changelist.step_seq) 
          [obj.goal_pos, adj] = FootstepPlanner.stepSeq2GoalPos(data.step_seq);
          obj.adjusted_footsteps = adj;
          obj.needs_plan = true;
        elseif changelist.goal
          if (data.goal.is_new_goal || isempty(obj.old_steps))
            goal_pos = FootstepPlanner.decodePosition3d(data.goal.goal_pos);
            if ~any(isnan(goal_pos([1,2,6])))
              obj.adjusted_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
              msg ='Foot Plan : Received New Goal'; disp(msg); send_status(6,0,0,msg);
              obj.goal_pos = goal_pos;
              if data.goal.goal_type == drc.walking_goal_t.GOAL_TYPE_RIGHT_FOOT
                obj.goal_pos = footCenter2StepCenter(obj.biped, obj.goal_pos, true, obj.options.nom_step_width);
              elseif data.goal.goal_type == drc.walking_goal_t.GOAL_TYPE_LEFT_FOOT
                obj.goal_pos = footCenter2StepCenter(obj.biped, obj.goal_pos, false, obj.options.nom_step_width);
              end
              obj.needs_plan = true;
            end
          end
        elseif changelist.plan_commit
          obj.needs_plan = false;
          obj.goal_pos = [];
          msg ='Foot Plan : Committed'; disp(msg); send_status(6,0,0,msg);
        elseif changelist.plan_reject
          obj.goal_pos = [];
          obj.needs_plan = false;
          msg ='Foot Plan : Rejected'; disp(msg); send_status(6,0,0,msg);
        else
          obj.needs_plan = false;
        end
      else
        obj.needs_plan = false;
      end
    end

    function obj = getNewDraggedSteps(obj, data, ~, changelist)
      if changelist.plan_con
        % apply changes from user adjustment of footsteps in viewer
        new_X = FootstepPlanListener.decodeFootstepPlan(data.plan_con);
        new_X = new_X(1);
        matching_ndx = find([obj.steps.id] == new_X.id);
        if ~isempty(matching_ndx)
          obj.adjusted_footsteps(matching_ndx) = struct('pos', new_X.pos,'is_right_foot',new_X.is_right_foot);
        end
      end
    end

    function obj = applyStepAdjustments(obj)
      n_steps = length(obj.steps);
      function ndx = n2p(ndx)
        % convert negative footstep index (from end) to positive (from start)
        ndx(ndx < 0) = n_steps + ndx(ndx < 0) + 1;
      end
      function ndx = p2n(ndx)
        % convert positive (from start) to negative (from end)
        ndx(ndx > 0) = ndx(ndx > 0) - n_steps - 1;
      end

      if ~isempty(obj.adjusted_footsteps)
        for j = obj.adjusted_footsteps.keys()
          ndx = j{1};
          if n2p(ndx) == n_steps
            % Handle the edge case when the desired foot step sequence gives
            % the wrong step order for the final two steps
            if obj.adjusted_footsteps(ndx).is_right_foot ~= obj.steps(end).is_right_foot
              for k = [p2n(n_steps-1), n_steps-1]
                if obj.adjusted_footsteps.isKey(k)
                  swap = obj.adjusted_footsteps(ndx);
                  obj.adjusted_footsteps(ndx) = obj.adjusted_footsteps(k);
                  obj.adjusted_footsteps(k) = swap;
                  break;
                end
              end
            end
          end
        end
        for j = obj.adjusted_footsteps.keys()
          ndx = j{1};
          if ndx <= n_steps
            if obj.steps(n2p(ndx)).is_right_foot ~= obj.adjusted_footsteps(ndx).is_right_foot
              msg ='Foot Plan : Error: Mismatched foot'; disp(msg); send_status(6,0,0,msg);
              break;
            end
            old_X = obj.steps(n2p(ndx));
            new_pos = obj.biped.footOrig2Contact(obj.adjusted_footsteps(ndx).pos, 'center', old_X.is_right_foot);
            obj.steps(n2p(ndx)).pos = new_pos;
            if ~obj.options.ignore_terrain || isnan(obj.adjusted_footsteps(ndx).pos(3))
              obj.steps(n2p(ndx)).pos = fitStepToTerrain(obj.biped, obj.steps(n2p(ndx)).pos, 'center');
            end
          end
        end
      end
    end


    function run(obj, interval)
      obj.steps = [];
      obj.old_steps = [];
      obj.goal_pos = [];

      if nargin < 2
        interval = 1; % time between updates, s
      end
      
      data = struct('utime', 0);
      tic;
      while (1)
        [data, changed, changelist] = obj.updateData(data);
        has_required = isfield(data, 'x0');
        if ~has_required
          pause(interval);
          continue;
        end

        if isfield(data, 'goal') && data.goal.behavior == drc.walking_goal_t.BEHAVIOR_CRAWLING
          continue;
        end

        obj = obj.updateGoal(data, changed, changelist);
        % Generate new footsteps, if needed
        if obj.needs_plan
          assert(~isempty(obj.goal_pos));
          [obj.steps, ~] = obj.biped.createInitialSteps(data.x0, obj.goal_pos, obj.options);
        end

        obj = obj.getNewDraggedSteps(data, changed, changelist);
        obj = obj.applyStepAdjustments();
        [obj, published] = obj.doOutput(data, changed, changelist);
        if ~published
          pause(interval)
          fprintf(1, 'waiting... (t=%f)\n', toc);
        end
      end
    end

    function [obj, published] = doOutput(obj, data, changed, changelist)
      if isempty(obj.steps)
        modified = false;
      elseif isequal(size(obj.old_steps), size(obj.steps)) && all(all(abs([obj.old_steps.pos] - [obj.steps.pos]) < 0.005))
        modified = false;
      else
        modified = true;
      end

      published = false;
      if (modified || changelist.goal || changelist.step_seq) && ~isempty(obj.steps)
        published = true;
        obj.publishFoosteps(data, changed, changelist);
        obj.old_steps = obj.steps;
      end
    end
    
    function publishFoosteps(obj, data, ~, changelist)
      isnew = changelist.goal || changelist.step_seq || isempty(obj.old_steps);
      Xout = obj.steps;
      % Convert from foot center to foot origin
      for j = 1:length(obj.steps)
        Xout(j).pos = obj.biped.footContact2Orig(obj.steps(j).pos, 'center', obj.steps(j).is_right_foot);
      end
      obj.biped.publish_footstep_plan(Xout, data.utime, isnew, obj.options);
      msg ='Foot Plan : Published'; disp(msg); send_status(6,0,0,msg);
    end
  end

  methods (Static=true)
    function [goal_pos, adjusted_footsteps] = stepSeq2GoalPos(step_seq)
      [~, sort_ndx] = sort(step_seq.link_timestamps);
      adjusted_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
      for j = 1:step_seq.num_links
        ndx = -step_seq.num_links+j-1;
        adjusted_footsteps(ndx) = struct('pos', FootstepPlanner.decodePosition3d(step_seq.link_origin_position(sort_ndx(j))), 'is_right_foot', strcmp(step_seq.link_name(sort_ndx(j)), 'r_foot'));
      end
      goal_pos = zeros(6,1);
      goal_pos(1:3) = mean([adjusted_footsteps(-2).pos(1:3), adjusted_footsteps(-1).pos(1:3)], 2);
      goal_pos(4:6) = adjusted_footsteps(-2).pos(4:6) + 0.5 * angleDiff(adjusted_footsteps(-2).pos(4:6), adjusted_footsteps(-1).pos(4:6));
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


