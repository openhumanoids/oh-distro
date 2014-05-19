function test_MIQP()

options.floating = true;
options.dt = 0.001;

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
options.visual = false; % loads faster
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

request = drc.footstep_plan_request_t();
request.utime = 0;

fp = load(strcat(getenv('DRC_PATH'), '/control/matlab/data/atlas_fp.mat'));
fp.xstar(3) = fp.xstar(3) + 0.50; % make sure we're not assuming z = 0
request.initial_state = r.getStateFrame().lcmcoder.encode(0, fp.xstar);

r = r.setTerrain(KinematicTerrainMap(r, fp.xstar(1:r.getNumDOF), true));

foot_orig = struct('right', [0;-0.15;0;0;0;0], 'left', [0;0.15;0;0;0;0]);

stones = [0, 0.15, 0;
          0, -0.15, 0.005;
          0.5, 0.15, 0.01;
          0.5, -0.15, 0.02;
          1, 0.15, 0.03;
          1, -0.15, 0.04;
          1.5, -0.15, 0.05;
          1.5, 0.15, 0.06;
          2, 0, 0.07]';
safe_regions = struct('A', {}, 'b', {}, 'point', {}, 'normal', {});
if 1
  for j = 1:size(stones, 2)
    [Ai, bi] = poly2lincon(stones(1,j) + [-.15, -.15, .15, .15],...
                           stones(2,j) + [-.1, .1, .1, -.1]);
    Ai = [Ai, zeros(size(Ai, 1), 1)];
    safe_regions(end+1) = struct('A', Ai, 'b', bi, 'point', [0;0;stones(3,j)], 'normal', [0;0;1]);
  end
else
  [Ai, bi] = poly2lincon([-.2, -.2, 5,5], [-2, 2, 2, -2]);
  Ai = [Ai, zeros(size(Ai, 1), 1)];
  safe_regions(1) = struct('A', Ai, 'b', bi, 'point', [0;0;0], 'normal', [0;0;1]);
end

goal_pos = struct('right', [2;-0.15;0.1;0;0;0],...
                  'left',  [2;+0.15;0.1;0;0;0]);


request.params = drc.footstep_plan_params_t();
request.params.max_num_steps = 10;
request.params.min_num_steps = 0;
request.params.min_step_width = 0.18;
request.params.nom_step_width = 0.26;
request.params.max_step_width = 0.35;
request.params.nom_forward_step = 0.15;
request.params.max_forward_step = 0.4;
request.params.nom_upward_step = 0.25;
request.params.nom_downward_step = 0.15;
request.params.ignore_terrain = true;
request.params.planning_mode = drc.footstep_plan_params_t.MODE_AUTO;
request.params.behavior = drc.footstep_plan_params_t.BEHAVIOR_BDI_STEPPING;
request.params.map_command = 0;
request.params.leading_foot = drc.footstep_plan_params_t.LEAD_LEFT;


tic
profile on
nsteps = 30;
seed_plan = FootstepPlan.blank_plan(nsteps, [r.foot_bodies_idx.right, r.foot_bodies_idx.left], request.params, safe_regions);
seed_plan.footsteps(1).pos = foot_orig.right;
seed_plan.footsteps(2).pos = foot_orig.left;
plan = footstepMIQP(r, seed_plan, goal_pos, 3, 30);
profile viewer
toc

figure(1);
clf
nsteps = length(plan.footsteps);
r_ndx = 2:2:nsteps;
l_ndx = 1:2:nsteps;
steps = [plan.footsteps.pos];
plot(steps(1,r_ndx), steps(2, r_ndx), 'bo')
hold on
plot(steps(1,l_ndx), steps(2,l_ndx), 'ro')
plot(steps(1,:), steps(2,:), 'k:')
for j = 1:length(safe_regions)
  V = iris.thirdParty.polytopes.lcon2vert(safe_regions(j).A(:,1:2), safe_regions(j).b);
  k = convhull(V(:,1), V(:,2));
  patch(V(k,1), V(k,2), 'k', 'FaceAlpha', 0.2);
end
axis equal

% profile on
% tic
% plan = footstepMIQP(r, plan, goal_pos);
% toc
% profile viewer