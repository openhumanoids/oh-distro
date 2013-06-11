function runCrawlingPlanner()
  options.floating = true;
  options.dt = 0.001;
  r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
  d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_crawling.mat'));
  foot_spec(1).body_ind = findLinkInd(r,'l_foot');
  foot_spec(2).body_ind = findLinkInd(r,'r_foot');
  foot_spec(3).body_ind = findLinkInd(r,'l_hand');
  foot_spec(4).body_ind = findLinkInd(r,'r_hand');

  foot_spec(1).contact_pt = mean(getContactPoints(findLink(r,'l_foot'),'heel'));
  foot_spec(2).contact_pt = mean(getContactPoints(findLink(r,'r_foot'),'heel'));
  foot_spec(3).contact_pt = getContactPoints(findLink(r,'l_foot'),'palm');
  foot_spec(4).contact_pt = getContactPoints(findLink(r,'r_foot'),'palm');
  
  options.direction = 0;
  options.gait = 1;
  
  [qtraj,support_times,supports,S,s1,s2,comtraj,zmptraj] = crawlingPlan(r,d.x_nom,foot_spec,options)
end
