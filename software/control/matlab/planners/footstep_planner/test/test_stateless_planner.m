function test_stateless_planner()

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
options.visual = false; % loads faster
r = Atlas([],options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

request = drc.footstep_plan_request_t();
request.utime = 0;

fp = load(strcat(getenv('DRC_PATH'), '/control/matlab/data/atlas_fp.mat'));
fp.xstar(3) = fp.xstar(3) + 0.50; % make sure we're not assuming z = 0
request.initial_state = r.getStateFrame().lcmcoder.encode(0, fp.xstar);

r = r.setTerrain(DRCTerrainMap(false));

request = construct_default_request(request);

p = StatelessFootstepPlanner();

test_bad_step_width(p, r, request.copy())

test_terrain_height(p, r, request.copy())

test_single_goal_step(p, r, request.copy());

test_multi_goal_step(p, r, request.copy());

end

function request = construct_default_request(request)
request.goal_pos = drc.position_3d_t();
request.goal_pos.translation = drc.vector_3d_t();
request.goal_pos.translation.x = 2.0;
request.goal_pos.translation.y = 0;
request.goal_pos.translation.z = 0;
request.goal_pos.rotation = drc.quaternion_t();
request.goal_pos.rotation.w = 1.0;
request.goal_pos.rotation.x = 0;
request.goal_pos.rotation.y = 0;
request.goal_pos.rotation.z = 0;

request.num_goal_steps = 0;
request.num_existing_steps = 0;
request.num_iris_regions = 0;

request.params = drc.footstep_plan_params_t();
request.params.max_num_steps = 10;
request.params.min_num_steps = 0;
request.params.min_step_width = 0.18;
request.params.nom_step_width = 0.26;
request.params.max_step_width = 0.35;
request.params.nom_forward_step = 0.2;
request.params.max_forward_step = 0.35;
request.params.nom_upward_step = 0.25;
request.params.nom_downward_step = 0.15;
request.params.planning_mode = drc.footstep_plan_params_t.MODE_AUTO;
request.params.behavior = drc.footstep_plan_params_t.BEHAVIOR_BDI_STEPPING;
request.params.map_mode = drc.footstep_plan_params_t.FOOT_PLANE;
request.params.leading_foot = drc.footstep_plan_params_t.LEAD_LEFT;

request.default_step_params = drc.footstep_params_t();
request.default_step_params.utime = 0;
request.default_step_params.step_speed = 1.0;
request.default_step_params.step_height = 0.05;
request.default_step_params.constrain_full_foot_pose = false;
request.default_step_params.bdi_step_duration = 0;
request.default_step_params.bdi_sway_duration = 0;
request.default_step_params.bdi_lift_height = 0;
request.default_step_params.bdi_toe_off = drc.atlas_behavior_step_action_t.TOE_OFF_ENABLE;
request.default_step_params.bdi_knee_nominal = 0;
request.default_step_params.bdi_max_body_accel = 0;
request.default_step_params.bdi_max_foot_vel = 0;
request.default_step_params.bdi_sway_end_dist = 0.02;
request.default_step_params.bdi_step_end_dist = 0.02;
request.default_step_params.mu = 1.0;
end

function test_bad_step_width(planner, biped, request)

warning('off', 'DRC:Biped:BadNominalStepWidth');
request.params.nom_step_width = request.params.max_step_width;
planner.plan_footsteps(biped, request);

request.params.nom_step_width = request.params.min_step_width;
planner.plan_footsteps(biped, request);

request.params.max_step_width = request.params.min_step_width;
planner.plan_footsteps(biped, request);

end

function test_terrain_height(p, r, request)

plan = p.plan_footsteps(r, request);
plan.toLCM();
lc = lcm.lcm.LCM.getSingleton();
lc.publish('FOOTSTEP_PLAN_RESPONSE', plan.toLCM());
footsteps = plan.footsteps;
pos3 = footsteps(3).pos.inFrame(footsteps(3).frames.orig);
valuecheck(pos3(3), r.getTerrainHeight(pos3(1:2)) + 0.0811, 1e-3);
assert(length(footsteps) == 12);
assert(all([footsteps.infeasibility] < 1e-4))
end

function test_single_goal_step(p, r, request)

request.num_goal_steps = 1;
goal_steps = javaArray('drc.footstep_t', request.num_goal_steps);
goal_steps(1) = drc.footstep_t();
goal_steps(1).pos = drc.position_3d_t();
goal_steps(1).pos.translation = drc.vector_3d_t();
goal_steps(1).pos.translation.x = 2.0;
goal_steps(1).pos.translation.y = -0.15;
goal_steps(1).pos.translation.z = 0;
goal_steps(1).pos.rotation = drc.quaternion_t();
goal_steps(1).pos.rotation.w = 1.0;
goal_steps(1).pos.rotation.x = 0;
goal_steps(1).pos.rotation.y = 0;
goal_steps(1).pos.rotation.z = 0;
goal_steps(1).id = -1;
goal_steps(1).is_right_foot = 1;
goal_steps(1).fixed_z = true;
request.goal_steps = goal_steps;

plan = p.plan_footsteps(r, request);
footsteps = plan.footsteps;
s = Footstep.from_footstep_t(goal_steps(1), r);
valuecheck(footsteps(end).pos.inFrame(footsteps(end).frames.center).double(), s.pos.inFrame(s.frames.center).double());
end

function test_multi_goal_step(p, r, request)
request.num_goal_steps = 3;
goal_steps = javaArray('drc.footstep_t', request.num_goal_steps);
goal_steps(1) = drc.footstep_t();
goal_steps(1).pos = drc.position_3d_t();
goal_steps(1).pos.translation = drc.vector_3d_t();
goal_steps(1).pos.translation.x = 2.0;
goal_steps(1).pos.translation.y = -0.15;
goal_steps(1).pos.translation.z = 0;
goal_steps(1).pos.rotation = drc.quaternion_t();
goal_steps(1).pos.rotation.w = 1.0;
goal_steps(1).pos.rotation.x = 0;
goal_steps(1).pos.rotation.y = 0;
goal_steps(1).pos.rotation.z = 0;
goal_steps(1).id = -1;
goal_steps(1).is_right_foot = 1;
goal_steps(2) = drc.footstep_t();
goal_steps(2).pos = drc.position_3d_t();
goal_steps(2).pos.translation = drc.vector_3d_t();
goal_steps(2).pos.translation.x = 2.1;
goal_steps(2).pos.translation.y = 0.1;
goal_steps(2).pos.translation.z = 0;
goal_steps(2).pos.rotation = drc.quaternion_t();
goal_steps(2).pos.rotation.w = 1.0;
goal_steps(2).pos.rotation.x = 0;
goal_steps(2).pos.rotation.y = 0;
goal_steps(2).pos.rotation.z = 0;
goal_steps(2).id = -1;
goal_steps(2).is_right_foot = 0;
goal_steps(3) = drc.footstep_t();
goal_steps(3).pos = drc.position_3d_t();
goal_steps(3).pos.translation = drc.vector_3d_t();
goal_steps(3).pos.translation.x = 2.2;
goal_steps(3).pos.translation.y = -0.15;
goal_steps(3).pos.translation.z = 0.2;
goal_steps(3).pos.rotation = drc.quaternion_t();
goal_steps(3).pos.rotation.w = 1.0;
goal_steps(3).pos.rotation.x = 0;
goal_steps(3).pos.rotation.y = 0;
goal_steps(3).pos.rotation.z = 0;
goal_steps(3).id = -1;
goal_steps(3).is_right_foot = 1;
goal_steps(3).fixed_z = true;
request.goal_steps = goal_steps;

plan = p.plan_footsteps(r, request);
footsteps = plan.footsteps;
assert(all([footsteps(1:2:end-1).body_idx] ~= [footsteps(2:2:end).body_idx]))
valuecheck(footsteps(end).pos(3), 0.2);

request.num_goal_steps = 3;
goal_steps = javaArray('drc.footstep_t', request.num_goal_steps);
goal_steps(1) = drc.footstep_t();
goal_steps(1).pos = drc.position_3d_t();
goal_steps(1).pos.translation = drc.vector_3d_t();
goal_steps(1).pos.translation.x = 2.0;
goal_steps(1).pos.translation.y = 0.1;
goal_steps(1).pos.translation.z = 0;
goal_steps(1).pos.rotation = drc.quaternion_t();
goal_steps(1).pos.rotation.w = 1.0;
goal_steps(1).pos.rotation.x = 0;
goal_steps(1).pos.rotation.y = 0;
goal_steps(1).pos.rotation.z = 0;
goal_steps(1).id = -1;
goal_steps(1).is_right_foot = 0;
goal_steps(2) = drc.footstep_t();
goal_steps(2).pos = drc.position_3d_t();
goal_steps(2).pos.translation = drc.vector_3d_t();
goal_steps(2).pos.translation.x = 2.1;
goal_steps(2).pos.translation.y = -0.15;
goal_steps(2).pos.translation.z = 0;
goal_steps(2).pos.rotation = drc.quaternion_t();
goal_steps(2).pos.rotation.w = 1.0;
goal_steps(2).pos.rotation.x = 0;
goal_steps(2).pos.rotation.y = 0;
goal_steps(2).pos.rotation.z = 0;
goal_steps(2).id = -1;
goal_steps(2).is_right_foot = 1;
goal_steps(3) = drc.footstep_t();
goal_steps(3).pos = drc.position_3d_t();
goal_steps(3).pos.translation = drc.vector_3d_t();
goal_steps(3).pos.translation.x = 2.2;
goal_steps(3).pos.translation.y = 0.1;
goal_steps(3).pos.translation.z = 0;
goal_steps(3).pos.rotation = drc.quaternion_t();
goal_steps(3).pos.rotation.w = 1.0;
goal_steps(3).pos.rotation.x = 0;
goal_steps(3).pos.rotation.y = 0;
goal_steps(3).pos.rotation.z = 0;
goal_steps(3).id = -1;
goal_steps(3).is_right_foot = 0;
request.goal_steps = goal_steps;

plan = p.plan_footsteps(r, request);
footsteps = plan.footsteps;
assert(all([footsteps(1:2:end-1).body_idx] ~= [footsteps(2:2:end).body_idx]))
end
