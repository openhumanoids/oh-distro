function [lambdas, feasibility2, foot_centers] = scanWalkingTerrain(biped, traj, current_pos, nom_step_width)
% Scan out a grid of terrain along the robot's planned walking trajectory, then filter that terrain by its acceptability for stepping on. 
% @param traj either a BezierTraj or a DirectTraj
% @param current_pos where the center of the robot's feet currently is
% @retval lambdas linearly spaced points between 0 and 1 representing positions along the trajectory
% @retval feasibility a struct with 'right' and 'left' fields. If feasibility.right(n) == 1, then the right foot can be safely placed at the location along the trajectory given by lambdas(n)
% @retval foot_centers the center position of each foot at each lambda 

debug = false;

if nargin < 4
  nom_step_width = biped.nom_step_width;
end

foot_radius = sqrt(sum((biped.foot_contact_offsets.right.toe - biped.foot_contact_offsets.right.center).^2)) + 0.00;

lambdas = linspace(0, 1, 200);
traj_poses = traj.eval(lambdas);
if any(any(isnan(traj_poses)))
  error(['Got bad footstep trajectory data (NaN) at time: ', datestr(now())]);
end
foot_centers = struct('right', biped.stepCenter2FootCenter(traj_poses, 1, nom_step_width),...
                      'left', biped.stepCenter2FootCenter(traj_poses, 0, nom_step_width));

feas_check = biped.getTerrain().getStepFeasibilityChecker(foot_radius, struct('debug', false));
for f = {'left', 'right'}
  ft = f{1};
  feasibility2.(ft) = feas_check(foot_centers.(ft)(1:2,:));
end

end