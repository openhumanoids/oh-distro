function playbackPDFFTrajectory(q_traj,support_times,supports,simrate)
typecheck(q_traj,'Trajectory');
typecheck(supports,'SupportState');

% todo: makeFist?

options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

nq = getNumDOF(r);
nu = getNumInputs(r);

% first put it in the intiial condition, and wait
q0 = eval(q_traj,q_traj.tspan(1));
q0(3)=-10; % artificially put the robot below the ground so that all supports are active
[u0,r] = inverseDynamics(r,q0,0*q0,0*q0,supports(1));
actuated = getActuatedJoints(r);
fr = AtlasPositionRef(r,'crawling',4);
publish(fr,0,[q0(actuated);zeros(nu,1);u0],defaultChannel(fr));

qdot_traj = fnder(q_traj);
qddot_traj = fnder(qdot_traj);
  
breaks = getBreaks(q_traj);
u = zeros(getNumInputs(r),length(breaks));
for i=1:length(breaks)
  t = breaks(i);
  supp_idx = find(support_times<=t,1,'last');
  active_supports = supports(supp_idx);
  u(:,i) = inverseDynamics(r,eval(q_traj,t),eval(qdot_traj,t),eval(qddot_traj,t),active_supports);
end

q_actuated_traj = q_traj(actuated);
qdot_actuated_traj = qdot_traj(actuated);
u_traj = PPTrajectory(spline(breaks,u));

command_traj = setOutputFrame([q_actuated_traj;qdot_actuated_traj;u_traj],fr);

options.realtime_factor = simrate;
options.tspan = command_traj.tspan;
runLCM(command_traj,[],options);

end