classdef StepRecoveryPlanner < DRCPlanner
  properties
    biped
  end

  methods
    function obj = StepRecoveryPlanner(biped)
      obj = obj@DRCPlanner();
      obj.biped = biped;
      obj = addInput(obj, 'x0', 'EST_ROBOT_STATE', obj.biped.getStateFrame().lcmcoder, 1, 1, 0);
      obj = addInput(obj, 'foot_contact', 'FOOT_CONTACT_ESTIMATE', 'drc.foot_contact_estimate_t', 1, 1, 0);
      obj = addInput(obj, 'stop_walking', 'BRACE_FOR_FALL', 'drc.utime_t', 0, 1, 1);
      obj = addInput(obj, 'stop_walking', 'STOP_WALKING', 'drc.plan_control_t', 0, 1, 1);
      % obj = addInput(obj, 'push', 'ATLAS_PUSH', 'drc.atlas_push_t', 0,1,1);
    end


    function plan(obj, data)
      % profile on
      planning_time = getLastTimestamp(obj, 1)/1e6;
      if data.foot_contact.left_contact || data.foot_contact.right_contact
        contact_state = data.foot_contact;
      else
        contact_state = struct('right_contact', true, 'left_contact', true);
      end
      nq = getNumDOF(obj.biped);
      q = data.x0(1:nq);

      footsteps = recoverySteps(obj.biped, data.x0, contact_state);

      for j = 1:length(footsteps)
        if j > 2
          pos = fitStepToTerrain(obj.biped, footsteps(j).pos, 'center');
        else
          pos = footsteps(j).pos;
        end
        % footsteps(j).id = obj.biped.getNextStepID();
        footsteps(j).pos = obj.biped.footContact2Orig(pos, 'center', footsteps(j).is_right_foot);
      end
      footstep_opts = struct('ignore_terrain', 1, 'mu', 1, 'behavior', drc.footstep_opts_t.BEHAVIOR_WALKING);
      obj.biped.publish_footstep_plan(footsteps, etime(clock,[1970 1 1 0 0 0])*1000000, 1, footstep_opts);
      [support_times, supports, comtraj, foottraj, V, zmptraj] = walkingPlanFromSteps(obj.biped, data.x0, footsteps, footstep_opts);
      s1dot = fnder(V.s1,1);
      s2dot = fnder(V.s2,1);
      link_constraints = buildLinkConstraints(obj.biped, q, foottraj, []);
      d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      xstar = d.xstar;     
      qstar = xstar(1:nq);
      walking_plan = struct('S',V.S,'s1',V.s1,'s2',V.s2,'s1dot',s1dot,'s2dot',s2dot,...
          'support_times',support_times,'supports',{supports},'comtraj',comtraj,'mu',footstep_opts.mu,'t_offset',planning_time,...
          'link_constraints',link_constraints,'zmptraj',zmptraj,'qtraj',qstar,'ignore_terrain',footstep_opts.ignore_terrain);
      walking_pub = WalkingPlanPublisher('WALKING_PLAN');
      walking_pub.publish(planning_time*1e6,walking_plan);
      % profile viewer
    end
  end
end
