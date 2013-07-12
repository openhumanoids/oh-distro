function runKinematicFootContactEstimator
options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = setTerrain(r,DRCTerrainMap());
r = compile(r);

nq = getNumDOF(r);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');

contact_est = drc.foot_contact_estimate_t();
contact_est.detection_method = 0;

contact_threshold = 0.002; % m

lfoot_idx = findLinkInd(r,'l_foot');
rfoot_idx = findLinkInd(r,'r_foot');

lc = lcm.lcm.LCM.getSingleton();

x=[];
while isempty(x) % wait for initial state
  [x,t] = getNextMessage(state_frame,10);
end

while true 
  [x_,t_] = getNextMessage(state_frame,10);
  if (~isempty(x_))
    x=x_;
    t=t_;
  end
  contact_est.utime = t*1000000;
  
	q = x(1:nq); 
  kinsol = doKinematics(r,q,false,true);
    
  % get active contacts
  phi = contactConstraints(r,kinsol,[lfoot_idx,rfoot_idx]);

  % if any foot point is in contact, all contact points are active
  if any(phi(1:4)<contact_threshold)
    contact_est.left_contact = 1;
  else
    contact_est.left_contact = 0;
  end
  
  if any(phi(5:8)<contact_threshold)
    contact_est.right_contact = 1;
  else
    contact_est.right_contact = 0;
  end
  
  lc.publish('FOOT_CONTACT_ESTIMATE', contact_est);

end
