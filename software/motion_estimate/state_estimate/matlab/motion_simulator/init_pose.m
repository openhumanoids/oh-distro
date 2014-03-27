function pose = init_pose()

pose.utime = 0;
pose.P_l = zeros(3,1);
pose.V_l = zeros(3,1);
pose.lQb = [1;0;0;0];
pose.R = eye(3);%  depreciated if possible
pose.a_l = zeros(3,1);
pose.f_l = zeros(3,1);
pose.da = zeros(3,1);% to be depreciated
pose.q = [1;0;0;0];% to be depreciated
pose.E = zeros(3,1);% to be depreciated
pose.w_l = zeros(3,1);