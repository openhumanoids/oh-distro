classdef StatelessWalkingPlanner
  methods
    function obj = StatelessWalkingPlanner()
    end
  end

  methods(Static=true)
    function walking_plan = plan_walking(r, request)
      debug = false;
      
      x0 = r.getStateFrame().lcmcoder.decode(request.initial_state);
      q0 = x0(1:end/2);
      nq = getNumDOF(r);

      if request.use_new_nominal_state
        xstar = r.getStateFrame().lcmcoder.decode(request.new_nominal_state);
      else
        d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
        xstar = d.xstar;     
      end

      r = r.setInitialState(xstar); % TODO: do we need this? -robin
      qstar = xstar(1:nq);

      footstep_plan = FootstepPlan.from_footstep_plan_t(request.footstep_plan);
      footsteps = footstep_plan.footsteps;

      % Align the first two steps to the current feet poses
      feet_pos = feetPosition(r, q0);
      if footsteps(1).is_right_foot
        footsteps(1).pos = feet_pos.right; footsteps(2).pos = feet_pos.left;
      else
        footsteps(1).pos = feet_pos.left; footsteps(2).pos = feet_pos.right;
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

      [support_times, supports, comtraj, foottraj, V, zmptraj] = walkingPlanFromSteps(r, x0, footsteps);
      tf = comtraj.tspan(end); assert(abs(eval(V,tf,zeros(4,1)))<1e-4);  % relatively fast check to make sure i'm in the correct frame (x-zmp_tf)
      link_constraints = buildLinkConstraints(r, q0, foottraj, fixed_links);

      % compute s1,s2 derivatives for controller Vdot computation
      s1dot = fnder(V.s1,1);
      s2dot = fnder(V.s2,1);
      
      mus = zeros(length(footsteps), 1);
      for j = 1:length(footsteps)
        mus(j) = footsteps(j).walking_params.mu;
      end
      mu = mean(mus); % TODO: controller should accept step-specific mu
      walking_plan = struct('S',V.S,'s1',V.s1,'s2',V.s2,'s1dot',s1dot,'s2dot',s2dot,...
          'support_times',support_times,'supports',{supports},'comtraj',comtraj,'mu',mu,'t_offset',0,...
          'link_constraints',link_constraints,'zmptraj',zmptraj,'qtraj',qstar,'ignore_terrain',false)
      walking_pub = WalkingPlanPublisher('WALKING_PLAN');
      walking_pub.publish(0,walking_plan);

      [xtraj, ~, ~, ts] = robotWalkingPlan(r, q0, qstar, zmptraj, comtraj, link_constraints);
      % publish robot plan
      joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
      plan_pub = drc.control.RobotPlanPublisher(joint_names,true,'CANDIDATE_ROBOT_PLAN');

      plan_pub.publish(ts,xtraj);
      
      if debug
        tt = 0:0.05:ts(end);
        compoints = zeros(3,length(tt));
        for i=1:length(tt)
          compoints(1:2,i) = comtraj.eval(tt(i));
        end
        compoints(3,:) = getTerrainHeight(r,compoints(1:2,:));
        plot_lcm_points(compoints',[zeros(length(tt),1), ones(length(tt),1), zeros(length(tt),1)],555,'Desired COM',1,true);
      end  
    end
  end
end
