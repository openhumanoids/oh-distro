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
      footsteps = footstep_plan.footsteps;

      % Align the first two steps to the current feet poses
      feet_centers = feetPosition(r, q0);
      if footsteps(1).body_idx == r.foot_bodies_idx.right
        footsteps(1).pos = Point(footsteps(1).frames.center, feet_centers.right);
        footsteps(2).pos = Point(footsteps(2).frames.center, feet_centers.left);
      else
        footsteps(1).pos = Point(footsteps(1).frames.center, feet_centers.left);
        footsteps(2).pos = Point(footsteps(2).frames.center, feet_centers.right);
      end

      % Slow down the first and last steps, if necessary
      for j = [length(footsteps)-1,length(footsteps)]
        footsteps(j).walking_params.step_speed = min([footsteps(j).walking_params.step_speed, 1.5]);
      end

      fixed_links = [];
      if request.fix_right_hand
        fixed_links(end+1) = struct('link',r.findLinkInd('r_hand+r_hand_point_mass'),'pt',[0;0.1;0],'tolerance',0.05);
      end
      if request.fix_left_hand
        fixed_links(end+1) = struct('link',r.findLinkInd('l_hand+l_hand_point_mass'),'pt',[0;0.1;0],'tolerance',0.05);
      end

      [support_times, supports, comtraj, foottraj, V, zmptraj,c] = walkingPlanFromSteps(r, x0, footsteps);
      tf = comtraj.tspan(end); assert(abs(eval(V,tf,zeros(4,1)))<1e-4);  % relatively fast check to make sure i'm in the correct frame (x-zmp_tf)

      link_constraints = buildLinkConstraints(r, q0, foottraj, fixed_links);

      % % compute s1,s2 derivatives for controller Vdot computation
      % s1dot = fnder(V.s1,1);
      % s2dot = fnder(V.s2,1);

      mus = zeros(length(footsteps), 1);
      for j = 1:length(footsteps)
        mus(j) = footsteps(j).walking_params.mu;
      end
      mu = mean(mus); % TODO: controller should accept step-specific mu
      t_offset = 0;
      ignore_terrain = false;

      if ~compute_xtraj
        disp('Walk Plan: computing controller data')
        walking_plan = WalkingControllerData(V, support_times,...
                                           {supports}, comtraj, mu, t_offset,...
                                           link_constraints, zmptraj, qstar,...
                                           ignore_terrain,c);
      else
        [xtraj, ~, ~, ts] = robotWalkingPlan(r, q0, qstar, zmptraj, comtraj, link_constraints);
        joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
        joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'

        walking_plan = WalkingPlan(ts, xtraj, joint_names);
      end
      disp('done')
    end
  end
end
