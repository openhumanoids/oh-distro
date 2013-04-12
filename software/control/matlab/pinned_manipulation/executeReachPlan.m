function executeReachPlan(r,state_frame,xtraj,ts)
T = max(ts);
x0 = getInitialState(r); 
ind = getActuatedJoints(r);

qd_frame =  PinnedAtlasPositionRef(r);
des_traj = setOutputFrame(PPTrajectory(spline(ts,xtraj(ind,:))),qd_frame);

nq = r.getNumDOF();
q_breaks = xtraj(1:nq,:);
n_breaks = size(q_breaks,2);

l_hand = r.findLinkInd('l_hand');
r_hand = r.findLinkInd('r_hand');
l_hand_pt = [0;0;0];
r_hand_pt = [0;0;0];
l_hand_pos = zeros(7,n_breaks); % use quaternions
r_hand_pos = zeros(7,n_breaks); 
for i = 1:n_breaks
    kinsol = doKinematics(r,q_breaks(:,i));
    l_hand_pos(:,i) = forwardKin(r,kinsol,l_hand,l_hand_pt,2);
    r_hand_pos(:,i) = forwardKin(r,kinsol,r_hand,r_hand_pt,2);
end
% execute plan
disp('Executing plan...');
t=0;
t_offset = -1;
%texec=[];
while t <= T
    dt = 1e-3;
  [x,tsim] = getNextMessage(state_frame,50);
  base_curr = x(1:6);
  if (~isempty(x) && (t_offset == -1 || t <= T))
    if (t_offset == -1)
      t_offset = tsim;
    end
    t=tsim-t_offset;
    disp(t);
    l_hand_des = pose_spline(ts,l_hand_pos,t);
    r_hand_des = pose_spline(ts,r_hand_pos,t);
    %texec(end+1)=t;
    qd=des_traj.eval(t+dt);
    q_des = [base_curr;qd];
    q_curr = x(1:nq);
    q_out = online_planning(r,l_hand_des,r_hand_des,q_curr,q_des,l_hand,r_hand,l_hand_pt,r_hand_pt);
    
    qd_frame.publish(t,q_out,'JOINT_COMMANDS');
  end
end
end

function q_out = online_planning(r,l_hand_des,r_hand_des,q_curr,q_des,l_hand,r_hand,l_hand_pt,r_hand_pt)
    kinsol_curr = doKinematics(r,q_curr);
    [l_hand_pos,J_l_hand] = forwardKin(r,kinsol_curr,l_hand,l_hand_pt,2);
    [r_hand_pos,J_r_hand] = forwardKin(r,kinsol_curr,r_hand,r_hand_pt,2);
    ee_error = [l_hand_pos;r_hand_pos]-[l_hand_des;r_hand_des];
    J_curr = [J_l_hand;J_r_hand];
    J_curr = J_curr(:,7:end);
    
    lambda = 1e-1;
    q_error = lambda*pinv(J_curr)*ee_error;
    disp(max(q_error))
    %J_curr\ee_error;
    % gives   Warning: Rank deficient, rank = 12, tol =  1.166121e-15. 

    q_out = q_des(7:end)+q_error;
end
% [N,X]=hist(diff(texec),20);plot(X,N,'b.-')
