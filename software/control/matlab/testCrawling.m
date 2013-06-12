function testCrawling()
  options.floating = true;
  options.dt = 0.001;
  r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
  d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/suppine_crawl2.mat'));

  body_spec.body_ind = findLinkInd(r,'pelvis');
  body_spec.pt = zeros(3,1);
  
  foot_spec(1).body_ind = findLinkInd(r,'l_hand');
  foot_spec(2).body_ind = findLinkInd(r,'r_hand');
  foot_spec(3).body_ind = findLinkInd(r,'r_foot');
  foot_spec(4).body_ind = findLinkInd(r,'l_foot');

  foot_spec(1).contact_pt_ind = 1;
  foot_spec(2).contact_pt_ind = 1;
  [~,foot_spec(3).contact_pt_ind] = getContactPoints(findLink(r,'r_foot'),'heel');
  [~,foot_spec(4).contact_pt_ind] = getContactPoints(findLink(r,'l_foot'),'heel');
  
  options.direction = 0;
  options.step_length = -.2;
  options.gait = 0;
  
  [support_times,supports,V,comtraj,zmptraj,qdtraj] = crawlingPlan(r,d.x0,body_spec,foot_spec,options)
  
  qdtraj = setOutputFrame(qdtraj,AtlasPositionRef(r,'crawling'));
  
  options.realtime_factor = .12;
  options.tspan = qdtraj.tspan;
  runLCM(qdtraj,[],options);
  
end
