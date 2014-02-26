classdef FootstepPlanner < DRCPlanner
  properties
    biped
    monitors
    hmap_ptr
    options
    goal_pos
    adjusted_footsteps
    target_footsteps
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
      % mfallon modification to run BDI estimator seperate from state estimation: (tempoary - should be used in final system)
      % obj = addInput(obj,'x0','EST_ROBOT_STATE_BDI',obj.biped.getStateFrame().lcmcoder,true,true,false);
      obj = addInput(obj,'x0','EST_ROBOT_STATE',obj.biped.getStateFrame().lcmcoder,true,true,false);
      obj = addInput(obj, 'plan_con', 'FOOTSTEP_PLAN_CONSTRAINT', drc.deprecated_footstep_plan_t(), false, true,false);
      obj = addInput(obj, 'plan_commit', 'COMMITTED_FOOTSTEP_PLAN', drc.deprecated_footstep_plan_t(), false, true,false);
      obj = addInput(obj, 'plan_reject', 'REJECTED_FOOTSTEP_PLAN', drc.deprecated_footstep_plan_t(), false, true,false);
      obj = addInput(obj, 'step_seq', 'DESIRED_FOOT_STEP_SEQUENCE', drc.traj_opt_constraint_t(), false, true,true);
      obj.goal_pos = [];
      obj.adjusted_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
      obj.target_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');

      obj.defaults = struct('max_num_steps', 10,...
                            'min_num_steps', 0,...
                            'timeout', inf,...
                            'step_height', 0.05,...
                            'step_speed', 1.5,...
                            'min_step_width', 0.21,...
                            'nom_step_width', 0.26,...
                            'max_step_width', 0.40,...
                            'max_forward_step', 0.5,...
                            'nom_forward_step', 0.15,...
                            'follow_spline', false,...
                            'ignore_terrain', false,...
                            'right_foot_lead', true,...
                            'mu', 1,...
                            'behavior', drc.walking_goal_t.BEHAVIOR_BDI_STEPPING,...
                            'bdi_step_duration', 0.6,...
                            'bdi_sway_duration', 0,...
                            'bdi_lift_height', 0,...
                            'bdi_toe_off', 1,...
                            'bdi_knee_nominal', 0,...
                            'bdi_max_body_accel', 0,...
                            'bdi_max_foot_vel',0,...
                            'bdi_sway_end_dist',-1,...
                            'bdi_step_end_dist',-1,...
                            'map_command', drc.map_controller_command_t.FULL_HEIGHTMAP,...
                            'allow_optimization',false,...
                            'force_to_sticky_feet',false,...
                            'velocity_based_steps', false); 

      obj.needs_plan = false;
    end

    function obj = updateGoal(obj, data, changed, changelist)
      if changed
        if isfield(data, 'goal'); info = struct(data.goal); else info = struct(); end
        for x = fieldnames(obj.defaults)'
          if isfield(info, x{1}) && ~isnan(info.(x{1}))
            if ~isfield(obj.options, x{1}) || obj.options.(x{1}) ~= info.(x{1});
              obj.options.(x{1}) = info.(x{1});
              if ~isempty(obj.goal_pos); obj.needs_plan = true; end
              if strcmp(x{1}, 'right_foot_lead')
                obj.adjusted_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
                obj.target_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
              end
            end
          elseif isfield(obj.defaults, x{1})
            obj.options.(x{1}) = obj.defaults.(x{1});
          elseif isfield(obj.biped, x{1})
            obj.options.(x{1}) = obj.biped.(x{1});
          end
        end
        terrain = obj.biped.getTerrain();
        if ismethod(terrain, 'setMapMode')
          obj.biped.setTerrain(terrain.setMapMode(obj.options.map_command));
        end
%         obj.biped.setTerrain(obj.biped.getTerrain().setMapMode(obj.options.map_command));
        if (changelist.step_seq) 
          [obj.goal_pos, adj] = obj.stepSeq2GoalPos(data.step_seq);
          obj.target_footsteps = adj;
          obj.adjusted_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
          obj.needs_plan = true;
        elseif changelist.goal
          if (data.goal.is_new_goal || isempty(obj.old_steps))
            obj.goal_pos = struct('center', FootstepPlanner.decodePosition3d(data.goal.goal_pos));
            if ~any(isnan(obj.goal_pos.center([1,2,6])))
              obj.adjusted_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
              obj.target_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
              msg ='Foot Plan : Received New Goal'; disp(msg); send_status(6,0,0,msg);
              if data.goal.goal_type == drc.walking_goal_t.GOAL_TYPE_RIGHT_FOOT
                obj.goal_pos.center = obj.biped.footCenter2StepCenter(obj.goal_pos.center, true, obj.options.nom_step_width);
              elseif data.goal.goal_type == drc.walking_goal_t.GOAL_TYPE_LEFT_FOOT
                obj.goal_pos.center = obj.biped.footCenter2StepCenter(obj.goal_pos.center, false, obj.options.nom_step_width);
              end
              obj.needs_plan = true;
            end
          end
        elseif changelist.plan_commit
          msg ='Foot Plan : Committed'; disp(msg); send_status(6,0,0,msg);
        elseif changelist.plan_reject
          obj.steps = [];
          obj.goal_pos = [];
          obj.needs_plan = false;
          obj.adjusted_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
          obj.target_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
          msg ='Foot Plan : Rejected'; disp(msg); send_status(6,0,0,msg);
        else
          obj.needs_plan = false;
        end
      else
        obj.needs_plan = false;
      end
    end

    function obj = getNewDraggedSteps(obj, data, ~, changelist)
      if changelist.plan_con && ~isempty(obj.steps)
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

      if ~isempty(obj.target_footsteps) || ~isempty(obj.adjusted_footsteps)
        for j = obj.target_footsteps.keys()
          ndx = j{1};
          if n2p(ndx) == n_steps
            % Handle the edge case when the desired foot step sequence gives
            % the wrong step order for the final two steps
            if obj.target_footsteps(ndx).is_right_foot ~= obj.steps(end).is_right_foot
              for k = [p2n(n_steps-1), n_steps-1]
                if obj.target_footsteps.isKey(k)
                  swap = obj.target_footsteps(ndx);
                  obj.target_footsteps(ndx) = obj.target_footsteps(k);
                  obj.target_footsteps(k) = swap;
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
        k = cell2mat(obj.target_footsteps.keys());
        for ndx = k
          if ndx <= n_steps
            if obj.steps(n2p(ndx)).is_right_foot ~= obj.target_footsteps(ndx).is_right_foot
              msg ='Foot Plan : Error: Mismatched foot'; disp(msg); send_status(6,0,0,msg);
              break;
            end
            new_pos = obj.target_footsteps(ndx).pos;
            if ~obj.options.ignore_terrain || isnan(obj.target_footsteps(ndx).pos(3))
              new_pos = fitStepToTerrain(obj.biped, new_pos, 'center');
            end
            reachable = all(obj.biped.checkStepReach(obj.steps(n2p(ndx)-1).pos, new_pos, obj.steps(n2p(ndx)-1).is_right_foot, obj.options) <= 0);
            if obj.options.force_to_sticky_feet || reachable
              obj.steps(n2p(ndx)).pos = new_pos;
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
          disp('creating steps')
          obj.steps = obj.biped.createInitialSteps(data.x0, obj.goal_pos, obj.options);
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

    function steps = applySwingTerrain(obj, steps)
      % For every step (excluding the initial foot positions) scan the terrain map from the previous foot position to determine the maximal terrain profile over the width of the foot.
      if isempty(steps) || length(steps) <= 2
        return
      end
      for j = 3:length(steps)
        [contact_width, ~, ~] = contactVolume(obj.biped, steps(j-2).pos, steps(j).pos, struct('nom_z_clearance', steps(j).step_height));
        steps(j).terrain_pts = sampleSwingTerrain(obj.biped, steps(j-2).pos, steps(j).pos, contact_width);
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
      Xout = applySwingTerrain(obj, Xout);
      % Convert from foot center to foot origin
      for j = 1:length(obj.steps)
        Xout(j).pos = obj.biped.footContact2Orig(obj.steps(j).pos, 'center', obj.steps(j).is_right_foot);
      end
      if obj.options.behavior == drc.walking_goal_t.BEHAVIOR_BDI_STEPPING || obj.options.behavior == drc.walking_goal_t.BEHAVIOR_BDI_WALKING
        obj.options.channel = 'CANDIDATE_BDI_FOOTSTEP_PLAN';
      else
        obj.options.channel = 'CANDIDATE_FOOTSTEP_PLAN';
      end
      obj.biped.publish_footstep_plan(Xout, data.utime, isnew, obj.options);
      if isnew 
        msg ='Foot Plan : Published'; disp(msg); send_status(6,0,0,msg);
      end
    end
    
    function [goal_pos, target_footsteps] = stepSeq2GoalPos(obj, step_seq)
      [~, sort_ndx] = sort(step_seq.link_timestamps);
      target_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
      for j = 1:step_seq.num_links
        ndx = -step_seq.num_links+j-1;
        pos = FootstepPlanner.decodePosition3d(step_seq.link_origin_position(sort_ndx(j)));
        is_right_foot = strcmp(step_seq.link_name(sort_ndx(j)), 'r_foot');
        pos = obj.biped.footOrig2Contact(pos, 'center', is_right_foot);
        if is_right_foot
          goal_pos.right = pos;
        else
          goal_pos.left = pos;
        end
        target_footsteps(ndx) = struct('pos', pos, 'is_right_foot', is_right_foot);
      end
    end
  end

  methods (Static=true)
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


