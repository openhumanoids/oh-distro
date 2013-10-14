classdef FootstepPlanner < DRCPlanner
  properties
    biped
    monitors
    hmap_ptr
    options
    goal_pos
    adjusted_footsteps
  end
  
  methods
    function obj = FootstepPlanner(biped)
      typecheck(biped, 'Biped');
      
      obj = obj@DRCPlanner();
      obj.biped = biped;

      obj = addInput(obj,'goal', 'WALKING_GOAL', 'drc.walking_goal_t', true, true, true);
      obj = addInput(obj,'x0','EST_ROBOT_STATE',obj.biped.getStateFrame().lcmcoder,true,true,false);
      obj = addInput(obj, 'plan_con', 'FOOTSTEP_PLAN_CONSTRAINT', drc.footstep_plan_t(), false, true,false);
      obj = addInput(obj, 'plan_commit', 'COMMITTED_FOOTSTEP_PLAN', drc.footstep_plan_t(), false, true,false);
      obj = addInput(obj, 'plan_reject', 'REJECTED_FOOTSTEP_PLAN', drc.footstep_plan_t(), false, true,false);
      obj = addInput(obj, 'step_seq', 'DESIRED_FOOT_STEP_SEQUENCE', drc.traj_opt_constraint_t(), false, true,true);
      obj.goal_pos = [];
      obj.adjusted_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
    end

    function X = updatePlan(obj, X, data, changed, changelist)
      needs_plan = false;
      info = struct(data.goal);
      for x = {'max_num_steps', 'min_num_steps', 'timeout', 'step_height', 'step_speed', 'nom_step_width', 'nom_forward_step', 'max_forward_step','follow_spline', 'ignore_terrain', 'right_foot_lead', 'mu', 'behavior'}
        if isfield(info, x{1}) && ~isnan(info.(x{1}))
          if ~isfield(obj.options, x{1}) || obj.options.(x{1}) ~= info.(x{1});
            obj.options.(x{1}) = info.(x{1});
            needs_plan = true;
          end
        elseif isfield(obj.biped, x{1})
          obj.options.(x{1}) = obj.biped.(x{1});
        end
      end

      if (changelist.step_seq) 
        [obj.goal_pos, adj] = FootstepPlanner.stepSeq2GoalPos(data.step_seq);
        obj.adjusted_footsteps = adj;
        needs_plan = true;
      elseif (data.goal.is_new_goal && (changelist.goal || isempty(X)))
        obj.adjusted_footsteps = containers.Map('KeyType','int32', 'ValueType', 'any');
        msg ='Foot Plan : Received New Goal'; disp(msg); send_status(6,0,0,msg);
        obj.goal_pos = FootstepPlanner.decodePosition3d(data.goal.goal_pos);
        if data.goal.goal_type == drc.walking_goal_t.GOAL_TYPE_RIGHT_FOOT
          obj.goal_pos = footCenter2StepCenter(obj.biped, obj.goal_pos, true, obj.options.nom_step_width);
        elseif data.goal.goal_type == drc.walking_goal_t.GOAL_TYPE_LEFT_FOOT
          obj.goal_pos = footCenter2StepCenter(obj.biped, obj.goal_pos, false, obj.options.nom_step_width);
        end
        needs_plan = true;
      end
      if ~isempty(obj.goal_pos) && needs_plan
        [X, ~] = obj.biped.createInitialSteps(data.x0, obj.goal_pos, obj.options);
      end

      function ndx = n2p(ndx)
        % convert negative footstep index (from end) to positive (from start)
        ndx(ndx < 0) = length(X) + ndx(ndx < 0) + 1;
      end
      function ndx = p2n(ndx)
        % convert positive (from start) to negative (from end)
        ndx(ndx > 0) = ndx(ndx > 0) - length(X) - 1;
      end


      if changelist.plan_con
        % apply changes from user adjustment of footsteps in viewer
        new_X = FootstepPlanListener.decodeFootstepPlan(data.plan_con);
        new_X = new_X(1);
        matching_ndx = find([X.id] == new_X.id);
        if ~isempty(matching_ndx)
          obj.adjusted_footsteps(matching_ndx) = struct('pos', new_X.pos,'is_right_foot',new_X.is_right_foot);
        end
      end

      if ~isempty(obj.adjusted_footsteps)
        for j = obj.adjusted_footsteps.keys()
          ndx = j{1};
          if n2p(ndx) == length(X)
            % Handle the edge case when the desired foot step sequence gives
            % the wrong step order for the final two steps
            if obj.adjusted_footsteps(ndx).is_right_foot ~= X(end).is_right_foot
              for k = [p2n(length(X)-1), length(X)-1]
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
          if ndx <= length(X)
            if X(n2p(ndx)).is_right_foot ~= obj.adjusted_footsteps(ndx).is_right_foot
              msg ='Foot Plan : Error: Mismatched foot'; disp(msg); send_status(6,0,0,msg);
              break;
            end
            old_X = X(n2p(ndx));
            new_pos = obj.biped.footOrig2Contact(obj.adjusted_footsteps(ndx).pos, 'center', old_X.is_right_foot);
            X(n2p(ndx)).pos = new_pos;
            if ~obj.options.ignore_terrain
              X(n2p(ndx)).pos = fitStepToTerrain(obj.biped, X(n2p(ndx)).pos, 'center');
            end
          end
        end
      end
    end

    
    function X = plan(obj,data)
      X_old = [];
      obj.options = struct();
      planning = true;

      while 1
        [data, changed, changelist] = obj.updateData(data);
        if changed
          planning = true;
        end
        isnew = changelist.goal || changelist.step_seq || isempty(X_old);
        if changelist.plan_reject 
          planning = false;
          msg ='Foot Plan : Rejected'; disp(msg); send_status(6,0,0,msg);
%           break;
        end
        if changelist.plan_commit
          planning = false;
          msg ='Foot Plan : Committed'; disp(msg); send_status(6,0,0,msg);
        end
        if data.goal.behavior == drc.walking_goal_t.BEHAVIOR_CRAWLING
          planning = false;
        end
        if planning
          % profile on
          X = obj.updatePlan(X_old, data, changed, changelist);
          if isempty(X)
            modified = false;
          elseif isequal(size(X_old), size(X)) && all(all(abs([X_old.pos] - [X.pos]) < 0.01))
            modified = false;
          else
            modified = true;
          end
        else
          modified = false;
        end

        if (modified || changelist.goal || changelist.step_seq) && ~(data.goal.behavior == drc.walking_goal_t.BEHAVIOR_CRAWLING) && ~isempty(X)
          publishFoosteps(obj, X, data.utime, isnew);
          X_old = X;
        else
          pause(0.25)
        end
      end
    end
    function publishFoosteps(obj, X, utime, isnew)
      Xout = X;
      % Convert from foot center to foot origin
      for j = 1:length(X)
        Xout(j).pos = obj.biped.footContact2Orig(X(j).pos, 'center', X(j).is_right_foot);
      end
      obj.biped.publish_footstep_plan(Xout, utime, isnew, obj.options);
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


