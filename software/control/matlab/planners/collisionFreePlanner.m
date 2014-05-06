function [xtraj,info,infeasible_constraint] = collisionFreePlanner(r,tspan,varargin)
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


  for nt = 2:7
    t = linspace(tspan(1),tspan(2),nt);
    [xtraj,info,infeasible_constraint] = collisionFreeIKTraj(r,t,varargin{:});
    if info < 10, break; end
  end

