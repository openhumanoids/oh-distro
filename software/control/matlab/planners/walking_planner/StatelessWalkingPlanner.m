classdef StatelessWalkingPlanner
  methods
    function obj = StatelessWalkingPlanner()
    end
  end

  methods(Static=true)
    function walking_plan = plan_walking(r, request, compute_xtraj)
      debug = false;

      x0 = r.getStateFrame().lcmcoder.decode(request.initial_state);
      q0 = x0(1:end/2);
      kinsol = doKinematics(r, q0);
      nq = getNumDOF(r);

      if request.use_new_nominal_state
        xstar = r.getStateFrame().lcmcoder.decode(request.new_nominal_state);
      else
        xstar = r.loadFixedPoint();
      end

      if request.footstep_plan.params.ignore_terrain
        r = r.setTerrain(KinematicTerrainMap(r, q0, true));
        r = compile(r);
      else
        terrain = r.getTerrain();
        if ismethod(terrain, 'setBackupTerrain')
          terrain = terrain.setBackupTerrain(r, q0);
          r = r.setTerrain(terrain);
          r = compile(r);
        end
      end

%       r = r.setInitialState(xstar); % TODO: do we need this? -robin
      qstar = xstar(1:nq);

      footstep_plan = FootstepPlan.from_footstep_plan_t(request.footstep_plan, r);

      % Align the first two steps to the current feet poses
      feet_centers = feetPosition(r, q0);
      if footstep_plan.footsteps(1).frame_id == r.foot_frame_id.right
        footstep_plan.footsteps(1).pos = feet_centers.right;
        footstep_plan.footsteps(2).pos = feet_centers.left;
      else
        footstep_plan.footsteps(1).pos = feet_centers.left;
        footstep_plan.footsteps(2).pos = feet_centers.right;
      end

      % Slow down the first and last steps, if necessary
      for j = [length(footstep_plan.footsteps)-1,length(footstep_plan.footsteps)]
        footstep_plan.footsteps(j).walking_params.step_speed = min([footstep_plan.footsteps(j).walking_params.step_speed, 1.5]);
      end

      % Compute the ZMP traj, COM traj, feet trajs, etc.
      walking_plan_data = planWalkingZMP(r, x0, footstep_plan);

      if request.fix_right_hand
        walking_plan_data = walking_plan_data.fix_link(r, kinsol, r.findLinkInd('r_hand+r_hand_point_mass'), [0; 0.1; 0], 0.05, 0);
      end
      if request.fix_left_hand
        walking_plan_data = walking_plan_data.fix_link(r, kinsol, r.findLinkInd('l_hand+l_hand_point_mass'), [0; 0.1; 0], 0.05, 0);
      end

      if compute_xtraj
        [xtraj, ~, ts] = planWalkingStateTraj(r, walking_plan_data, xstar);
        joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
        joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'

        walking_plan = WalkingPlan(ts, xtraj, joint_names);
      else
        walking_plan = WalkingControllerData.from_drake_walking_data(walking_plan_data, qstar);
      end

      % [support_times, supports, comtraj, foottraj, V, zmptraj,c] = walkingPlanFromSteps(r, x0, footsteps);
      % tf = comtraj.tspan(end); assert(abs(eval(V,tf,zeros(4,1)))<1e-4);  % relatively fast check to make sure i'm in the correct frame (x-zmp_tf)

      % link_constraints = buildLinkConstraints(r, q0, foottraj, fixed_links);

      % % % compute s1,s2 derivatives for controller Vdot computation
      % % s1dot = fnder(V.s1,1);
      % % s2dot = fnder(V.s2,1);

      % mus = zeros(length(footsteps), 1);
      % for j = 1:length(footsteps)
      %   mus(j) = footsteps(j).walking_params.mu;
      % end
      % mu = mean(mus); % TODO: controller should accept step-specific mu
      % t_offset = 0;
      % ignore_terrain = false;

      % if ~compute_xtraj
      %   disp('Walk Plan: computing controller data')
      %   walking_plan = WalkingControllerData(V, support_times,...
      %                                      {supports}, comtraj, mu, t_offset,...
      %                                      link_constraints, zmptraj, qstar,...
      %                                      ignore_terrain,c);
      % else
      %   [xtraj, ~, ~, ts] = robotWalkingPlan(r, q0, qstar, zmptraj, comtraj, link_constraints);
      %   joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      %   joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'

      %   walking_plan = WalkingPlan(ts, xtraj, joint_names);
      % end
      disp('done')
    end
  end
end
