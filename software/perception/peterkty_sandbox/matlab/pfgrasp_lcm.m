%% clear memory, screen, and close all figures
clear, clc, close all;
addpath_drake;
addpath_control;
javaaddpath('/home/drc/drc/software/build/share/java/lcmtypes_trackers.jar');
javaaddpath('/home/drc/drc/software/build/share/java/lcmtypes_vicon.jar');
javaaddpath('/home/drc/drc/software/build/share/java/lcmtypes_bot2-frames.jar');
javaaddpath('/home/drc/drc/software/build/share/java/lcmtypes_bot2-core.jar');

javaaddpath('/usr/local/share/java/lcm.jar');

%% parameter
addpath particle_filter
global bound hbound
bound = 1;
hbound = bound / 2;
trueX = [0.1,0.1,0.1]';
insim = false;

%% which hand
LR = 'R'
nx = 3;
ny = 2;
sigma_u = 0.10; % system noise

%% Number of time steps
T = 10000; 

%% Initialize first campose
campose = zeros(7,T);
if insim
  campose(:,1) = [0,0,0,1,0,0,0]';
else
  campose(:,1) = getCamposefromLCM(LR);
end
obs_error = deg2rad(5);

%% PDF of process noise and noise generator function 
nu = 3;                                           % size of the vector of process noise
p_sys_noise   = @(u) normpdf(u, 0, sigma_u);
gen_sys_noise = @(u) normrnd(0, sigma_u, nx, 1);         % sample from p_sys_noise (returns column vector)

%% Process equation x[k] = sys(k, x[k-1], u[k]);
sys = @(k, xkm1, uk) xkm1 + uk; % (returns column vector)

%%
pf.k               = 1;                   % initial iteration number
pf.Ns              = 1000;                 % number of particles
pf.w               = zeros(pf.Ns, T);     % weights
pf.particles       = zeros(nx, pf.Ns, T); % particles
pf.gen_x0          = @gen_x0;              % function for sampling from initial pdf p_x0
pf.p_yk_given_xk   = @p_yk_given_xk;       % function of the observation likelihood PDF p(y[k] | x[k])
pf.gen_sys_noise   = gen_sys_noise;       % function for generating system noise

%% allocate memory
xh = zeros(nx, T); 
yh = zeros(ny, T); 
x = zeros(nx,T);  y = zeros(ny,T);

%% Estimate state
for k = 2:T
   fprintf('Iteration = %d/%d\n',k,T);
   pf.k = k;
   
   if insim 
     y(:,k) = sim_generate_yk(campose(:,k), trueX);
   else
     
     [y(:,k), campose(:,k)] = getBearingAndCamposefromLCM(LR);
     %yk = yk from tld bearing angles
   end
   
   [xh(:,k), pf] = particle_filter(sys, y(:,k), pf, 'systematic_resampling', campose(:,k));   
   figure(1);
   scatter3(pf.particles(1,1:3:end,k), pf.particles(2,1:3:end,k), pf.particles(3,1:3:end,k), 3, pf.w(1:3:end,k));
   if insim
     hold on
     scatter3(trueX(1), trueX(2), trueX(3), 5, 1);
     hold off
   else
   end
   xlabel('x'); ylabel('y'); zlabel('z');
   axis equal
   view(-90,90)
   
   % draw particles
   lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'pf');
   lcmgl.glColor3f(1,0,1);
   for i=1:3:(pf.Ns)
     lcmgl.sphere(pf.particles(1:3, i, k),0.01,100,100);
   end
 
   % filtered observation
   xk = xh(:,k);
   fprintf('xh=%f %f %f\n', xk);
   yh(:,k) = [ atan((xk(1)-campose(1,k))/xk(3)); atan((xk(2)-campose(2,k))/xk(3))];
   
   lcmgl.glColor3f(1,0,0);
   lcmgl.sphere(xk(1:3),0.05,100,100);
   
   
   % control campose
   if k<T
       if insim
         %campose(:,k+1) = [xk(1),xk(2),0];
         campose(:,k+1) = [xk(1),0,0];
       else
         %control
         delta = (xk - campose(1:3,k));
         camUnitVec = inv(quat2dcm(campose(4:7,k)'));
         dx = camUnitVec(:,1)' * delta * camUnitVec(:,1);  % dx on camera frame wrt world
         dy = camUnitVec(:,2)' * delta * camUnitVec(:,2);  % dy on camera frame wrt world if we want to use
         new_campose = [campose(1:3,k) + dx ; campose(4:7,k)];  % camera orient stay the same
         
         sendNewCampose(new_campose, LR);
         fprintf('old campose: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n', campose(1:7,k));
         fprintf('new campose: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n', new_campose(1:7));
         
         lcmgl.glColor3f(1,1,0);
         lcmgl.sphere(new_campose,0.03,100,100);
         
       end
   end
   
   lcmgl.switchBuffers;
   pause();
end

figure(2);
title('object pose estimate');
plot3(xh(1,2:end), xh(2,2:end), xh(3,2:end));
for i=2:length(xh)
  text(xh(1,i), xh(2,i), xh(3,i), sprintf('%d', i));
end
xlabel('x'); ylabel('y'); zlabel('z');
axis equal

figure(3);
title('camera pose');
plot3(campose(1,3:end), campose(2,3:end), campose(3,3:end));
for i=3:length(xh)
  text(campose(1,i), campose(2,i), campose(3,i), sprintf('%d', i));
end
xlabel('x'); ylabel('y'); zlabel('z');
axis equal
