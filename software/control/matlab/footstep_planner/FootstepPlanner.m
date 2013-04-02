classdef FootstepPlanner < DRCPlanner
  properties
    biped
    monitors
    walking_goal_coder
    % plan_publisher
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
    end
    
    function X = plan(obj,data)
      X_old = [];
      goal_pos = [];
      options = struct();
      last_publish_time = now();
      while 1
        [data, changed, changelist] = obj.updateData(data);
        if (changelist.goal && data.goal.is_new_goal) || isempty(goal_pos)
          disp('got new goal');
          goal_pos = [data.goal.goal_pos.translation.x;
                      data.goal.goal_pos.translation.y;
                      data.goal.goal_pos.translation.z];
          [goal_pos(4), goal_pos(5), goal_pos(6)] = quat2angle([data.goal.goal_pos.rotation.w,...
                                            data.goal.goal_pos.rotation.x,...
                                            data.goal.goal_pos.rotation.y,...
                                            data.goal.goal_pos.rotation.z], 'XYZ');
          %%% HACK for DRC Qual 1 
          goal_pos(3) = 0;
          %%% end hack 

          [X, foot_goals] = obj.biped.createInitialSteps(data.x0, goal_pos);
          for x = {'max_num_steps', 'min_num_steps', 'timeout', 'time_per_step', 'yaw_fixed', 'is_new_goal'}
            options.(x{1}) = data.goal.(x{1});
          end
          options.timeout = options.timeout / 1000000;
          isnew = true;
        end
        if changelist.plan_reject 
          disp('rejected')
          break;
        end
        if changelist.plan_con
          new_X = FootstepPlanListener.decodeFootstepPlan(data.plan_con);
          new_X = new_X(1);
          new_X.pos = obj.biped.footOrig2Contact(new_X.pos, 'center', new_X.is_right_foot);
          X([X.id] == new_X.id) = new_X;
          t = num2cell(obj.biped.getStepTimes([X.pos]));
          [X.time] = t{:};
        end

        [X, outputflag] = updateRLFootstepPlan(obj.biped, X, foot_goals, options, @heightfun);
        if isequal(size(X_old), size(X)) && all(all(abs([X_old.pos] - [X.pos]) < 0.01))
          modified = false;
        else
          modified = true;
        end
        X_old = X;

        if modified || ((now() - last_publish_time) * 24 * 60 * 60 > 1)
          Xout = X;
          % Convert from foot center to foot origin
          for j = 1:length(X)
            Xout(j).pos = obj.biped.footContact2Orig(X(j).pos, 'center', X(j).is_right_foot);
          end
          publish(X, @heightfun);
          last_publish_time = now();
        end
      end


      function publish(X, htfun)
        obj.biped.publish_footstep_plan(X, htfun, data.utime, isnew);
        isnew = false;
      end

      function h = heightfun(xy)
        h = zeros(1, length(xy(1,:)));
      end
    end
  end
end


