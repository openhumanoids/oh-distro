classdef FootstepPlanner < DRCPlanner
  properties
    biped
    monitors
    hmap_ptr
    options
    z_adjust
  end
  
  methods
    function obj = FootstepPlanner(biped)
      typecheck(biped, 'Biped');
      
      robot_name = 'atlas';
      obj = obj@DRCPlanner();
      % obj = obj@DRCPlanner('NAV_GOAL_TIMED',JLCMCoder(NavGoalCoder(robot_name)));
      obj.biped = biped;

      obj = addInput(obj,'goal', 'WALKING_GOAL', 'drc.walking_goal_t', 1, 1, 1);
      obj = addInput(obj,'x0','EST_ROBOT_STATE',obj.biped.getStateFrame().lcmcoder,true,true);
      obj = addInput(obj, 'plan_con', 'FOOTSTEP_PLAN_CONSTRAINT', drc.footstep_plan_t(), false, true);
      obj = addInput(obj, 'plan_commit', 'COMMITTED_FOOTSTEP_PLAN', drc.footstep_plan_t(), false, true);
      obj = addInput(obj, 'plan_reject', 'REJECTED_FOOTSTEP_PLAN', drc.footstep_plan_t(), false, true);
      % obj.hmap_ptr = HeightMapHandle();
      % mapAPIwrapper(obj.hmap_ptr);
      obj.z_adjust = -0.003; % amount to adjust each foot position to compensate for heightmap and gazebo weirdness
    end

    function X = updatePlan(obj, X, data, changed, changelist)
      if changelist.goal || isempty(X)
        msg ='Foot Plan : Received Goal Info'; disp(msg); send_status(6,0,0,msg);
        for x = {'max_num_steps', 'min_num_steps', 'timeout', 'step_height', 'step_speed', 'follow_spline', 'is_new_goal', 'ignore_terrain', 'right_foot_lead'}
          obj.options.(x{1}) = data.goal.(x{1});
        end
        % obj.options.time_per_step = obj.options.time_per_step / 1e9;
        obj.options.timeout = obj.options.timeout / 1e6;
      end
      if (changelist.goal && (data.goal.is_new_goal || ~data.goal.allow_optimization)) || isempty(X)
        msg ='Foot Plan : Received New Goal'; disp(msg); send_status(6,0,0,msg);
        goal_pos = [data.goal.goal_pos.translation.x;
                    data.goal.goal_pos.translation.y;
                    data.goal.goal_pos.translation.z];
        [goal_pos(4), goal_pos(5), goal_pos(6)] = quat2angle([data.goal.goal_pos.rotation.w,...
                                          data.goal.goal_pos.rotation.x,...
                                          data.goal.goal_pos.rotation.y,...
                                          data.goal.goal_pos.rotation.z], 'XYZ');
        [X, foot_goals] = obj.biped.createInitialSteps(data.x0, goal_pos, obj.options);
      end
      if changelist.plan_con
        % apply changes from user adjustment of footsteps in viewer
        new_X = FootstepPlanListener.decodeFootstepPlan(data.plan_con);
        new_X = new_X(1);
        new_X.pos = obj.biped.footOrig2Contact(new_X.pos, 'center', new_X.is_right_foot);
        new_X.pos(3) = new_X.pos(3) - obj.z_adjust; % add back the 3mm we subtracted before publishing
        matching_ndx = find([X.id] == new_X.id);
        if ~isempty(matching_ndx)
          old_x = X(matching_ndx);
          X(matching_ndx) = new_X;
          X(matching_ndx).is_in_contact = old_x.is_in_contact;
          if obj.options.ignore_terrain
            X(matching_ndx).pos(3) = old_x.pos(3);
          else
            X(matching_ndx).pos = fitStepToTerrain(obj.biped, X(matching_ndx).pos);
          end
        end
      end
    end

    
    function X = plan(obj,data)
      X_old = [];
      goal_pos = [];
      obj.options = struct();
      % last_publish_time = now();
      isnew = true;
      planning = true;

      while 1
        [data, changed, changelist] = obj.updateData(data);
        if changed
          planning = true;
        end
        isnew = changelist.goal || isempty(X_old);
        if changelist.plan_reject 
          planning = false;
          msg ='Foot Plan : Rejected'; disp(msg); send_status(6,0,0,msg);
          break;
        end
        if changelist.plan_commit
          planning = false;
          msg ='Foot Plan : Committed'; disp(msg); send_status(6,0,0,msg);
        end
        if planning
%           profile on
          X = obj.updatePlan(X_old, data, changed, changelist);
          if isequal(size(X_old), size(X)) && all(all(abs([X_old.pos] - [X.pos]) < 0.01))
            modified = false;
          else
            modified = true;
          end
          X_old = X;
        else
          modified = false;
        end

        % if modified || ((now() - last_publish_time) * 24 * 60 * 60 > 1)
        if modified
          Xout = X;
          % Convert from foot center to foot origin
          for j = 1:length(X)
            Xout(j).pos = obj.biped.footContact2Orig(X(j).pos, 'center', X(j).is_right_foot);
            % move the planned steps down by 3mm (helps with force classification and compensates for gazebo issues)
            Xout(j).pos(3) = Xout(j).pos(3) + obj.z_adjust;
          end
          publish(Xout);
%           profile viewer
        else
          pause(1)
        end
      end


      function publish(X)
        if length(X) > 2
          [~, foottraj, ~] = obj.biped.planInitialZMPTraj(data.x0(1:obj.biped.getNumDOF), X);
          pts.right = foottraj.right.orig.eval(linspace(foottraj.right.orig.tspan(1),...
                                                         foottraj.right.orig.tspan(end)));
          pts.left = foottraj.left.orig.eval(linspace(foottraj.left.orig.tspan(1),...
                                                         foottraj.left.orig.tspan(end)));
%           pts.right = obj.biped.footOrig2Contact(pts.right, 'center', 1);
%           pts.left = obj.biped.footOrig2Contact(pts.left, 'center', 0);
          plot_lcm_points(pts.right', repmat([224/255, 116/255, 27/255], size(pts.right, 2), 1), 30, 'Right Foot Trajectory', 2, 1);
          plot_lcm_points(pts.left', repmat([27/255, 148/255, 224/255], size(pts.left, 2), 1), 31, 'Left Foot Trajectory', 2, 1);
        else
          plot_lcm_points([nan nan nan], [0 0 0], 30, 'Right Foot Trajectory', 2, 1);
          plot_lcm_points([nan nan nan], [0 0 0], 31, 'Left Foot Trajectory', 2, 1);
        end
        obj.biped.publish_footstep_plan(X, data.utime, isnew);
        msg ='Foot Plan : Published'; disp(msg); send_status(6,0,0,msg);
      end


    end
  end
end


