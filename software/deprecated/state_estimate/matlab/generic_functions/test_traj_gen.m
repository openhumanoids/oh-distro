


%% Generate a randomized trajectory before broadcasing LCM messages

param.gravity = 9.81; % this is in the forward left up coordinate frame
param.dt = 1E-3;

for n = 1:10
    
    traj = gen_traj(20000, param, 1);

end




