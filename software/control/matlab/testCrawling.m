function testCrawling()
  options.floating = true;
  options.dt = 0.001;
  r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
  d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/suppine_crawl2.mat'));
  nq = getNumDOF(r);
  nu = getNumInputs(r);
  
  body_spec.body_ind = findLinkInd(r,'pelvis');
  body_spec.pt = zeros(3,1);
  
  foot_spec(1).body_ind = findLinkInd(r,'l_hand');
  foot_spec(2).body_ind = findLinkInd(r,'r_hand');
  foot_spec(3).body_ind = findLinkInd(r,'r_foot');
  foot_spec(4).body_ind = findLinkInd(r,'l_foot');

  [~,foot_spec(1).contact_pt_ind] = getContactPoints(findLink(r,'l_hand'),'knuckle');
  [~,foot_spec(2).contact_pt_ind] = getContactPoints(findLink(r,'r_hand'),'knuckle');
  [~,foot_spec(3).contact_pt_ind] = getContactPoints(findLink(r,'r_foot'),'heel_mid');
  [~,foot_spec(4).contact_pt_ind] = getContactPoints(findLink(r,'l_foot'),'heel_mid');
  
  q0 = d.x0(1:nq); 
  q0(3)=-10; % artificially put the robot below the ground so that all supports are active
  u0 = inverseDynamics(r,q0,0*q0,0*q0,SupportState(r,[foot_spec.body_ind],{foot_spec.contact_pt_ind}));  
  actuated = getActuatedJoints(r);
  fr = AtlasPositionRef(r,'crawling',4);
  publish(fr,0,[q0(actuated);zeros(nu,1);u0],defaultChannel(fr));
  
  options.direction = 0;
  options.step_length = -.2;
  options.gait = 2;
  options.draw = false;
  
%  [support_times,supports,V,comtraj,zmptraj,qdtraj] = 
  [q_traj,support_times,supports] = crawlingPlan(r,d.x0,body_spec,foot_spec,options)
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
  
  options.realtime_factor = .12;
  options.tspan = q_actuated_traj.tspan;
  runLCM(command_traj,[],options);
  
end
