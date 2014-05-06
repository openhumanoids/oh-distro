function [xtraj,info,infeasible_constraint,xtraj_feasible,info_feasible] = collisionFreePlanner(r,tspan,q_seed_traj,q_nom_traj,varargin)
  % [xtraj, info] = % collisionFreePlanner(r,tspan,q_seed_traj,q_nom_traj,options,constr1,constr2,...,ikoptions)
  % 
  % @param options - [OPTIONAL] Structure that may contain the following fields
  %   * frozen_groups - Cell array of strings chosen from the following list
  %     - 'pelvis'
  %     - 'back'
  %     - 'l_arm'
  %     - 'r_arm'
  %     These indicate that the corresponding portion of the body is 'frozen'
  %     by posture constraints. Pairs of links between which collision is
  %     impossible for the given set of frozen groups will be ignored.
  %     ***********************************************************************
  %     ***********************************************************************
  %     *** NOTE: It is currently up to the user to make sure the groups    ***
  %     *** indicated by this field are actually constrained. This function ***
  %     *** will not add constraints and may yield trajectories with        ***
  %     *** collisions if groups which are listed as frozen are actually    ***
  %     *** unconstrained.  You've been warned.                             ***
  %     ***********************************************************************
  %     ***********************************************************************
  %   * visualize - Boolean indicating whether or not to draw debug visuals
  %
  %   This is where code for paralellizing the knot point loop could go.


  if isstruct(varargin{1})
    options = varargin{1};
    varargin(1) = [];
  else
    options = struct();
  end
  options.seed_switch_threshold = 0.0;
  options.position_cost = 0;
  options.acceleration_cost = 0;
  %q_seed_traj = ConstantTrajectory(q_seed_traj.eval(tspan(1)));
  for nt = 3:7
    t = linspace(tspan(1),tspan(2),nt);
    [xtraj_feasible,info_feasible,infeasible_constraint,additional_t_samples] = collisionFreeIKTraj(r,t,q_seed_traj,q_nom_traj,options,varargin{:});
    if info_feasible < 10, break; end
  end

  %xtraj = xtraj_feasible;
  options.additional_t_samples = additional_t_samples;
  options.acceleration_cost = 1e-6;
  options.position_cost = 1e-4;
  options.allow_ikoptions_modification = false;
  q_seed_traj = xtraj_feasible(1:r.getNumDOF());
  q_nom_traj = xtraj_feasible(1:r.getNumDOF());
  [xtraj,info,infeasible_constraint] = collisionFreeIKTraj(r,t,q_seed_traj,q_nom_traj,options,varargin{:});
  if info > 10, xtraj = xtraj_feasible; info = info_feasible; end;
