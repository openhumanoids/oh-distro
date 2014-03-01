function [] = truth_ground_truth()


if (firstpass==1)
       % fake the data structure so that the computation results are put in
       % the second location and then moved to the first location at the
       % end of this cycle
       n = 2;
   end
    
   traj{n}.P = traj{n-1}.P + 1*(0*V__ + traj{n-1}.V) * dt; 
   traj{n}.f_l = traj{n-1}.R * (accls(n,:)');
   traj{n}.V = traj{n-1}.V + traj{n}.f_l * dt;
   traj{n}.R = closed_form_DCM_farrell(-rates(n,:)',traj{n-1}.R,dt);
   traj{n}.T = traj{n-1}.T+dt;
   V__ = traj{n-1}.V;