
%% Deisgn filter to subdue noise in generated trajectory

d = fdesign.lowpass('Fp,Fst,Ap,Ast',3,5,0.5,40,100);
param.Hd = design(d,'equiripple');
%%
param.gravity = 9.81; % this is in the forward left up coordinate frame
param.dt = 1E-3;

for n = 1:10
    
    gen_traj(20000, param, 1);
%     pause(0.5);
end