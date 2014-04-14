%% clear memory, screen, and close all figures
clear, clc, close all;


%% parameter
addpath particle_filter
bound = 0.5;
hbound = bound / 2;
trueX = [0.1,0.1,0.1]';
insim = true;

nx = 3;
ny = 2;
sigma_u = sqrt(0.00003); % system noise

%% Number of time steps
T = 40; 

%% Initialize first campose
campose = zeros(3,T);
campose(:,1) = [0,0,0]';
obs_error = deg2rad(5);

%% Initial PDF
gen_x0 = @(x) rand(3,1)*bound-[hbound,hbound,0]'+ campose(:,1);               % sample from p_x0 (returns column vector)

%% Observation likelihood PDF p(y[k] | x[k])
p_yk_given_xk = @(k, yk, xk, camposek) normpdf(yk(1) - atan((xk(1)-camposek(1))/xk(3)), 0, obs_error) * normpdf(yk(2) - atan((xk(2)-camposek(2))/xk(3)), 0, obs_error); 
%p_obs_noise(yk - obs(k, xk, 0));

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
pf.gen_x0          = gen_x0;              % function for sampling from initial pdf p_x0
pf.p_yk_given_xk   = p_yk_given_xk;       % function of the observation likelihood PDF p(y[k] | x[k])
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
     %yk = yk from tld bearing angles
   end
   
   [xh(:,k), pf] = particle_filter(sys, y(:,k), pf, 'systematic_resampling', campose(:,k));   
   figure(1);
   scatter3(pf.particles(1,:,k), pf.particles(2,:,k), pf.particles(3,:,k), 3, pf.w(:,k));
   hold on
   scatter3(trueX(1), trueX(2), trueX(3), 5, 1);
   hold off
   xlabel('x'); ylabel('y'); zlabel('z');
   axis equal
   pause();
 
   % filtered observation
   xk = xh(:,k);
   yh(:,k) = [ atan((xk(1)-campose(1,k))/xk(3)); atan((xk(2)-campose(2,k))/xk(3))];
   
   % control campose
   if k<T
     %campose(:,k+1) = [xk(1),xk(2),0];
     campose(:,k+1) = [xk(1),0,0];
   end
end

figure(2);
title('object pose estimate');
plot3(xh(1,2:end), xh(2,2:end), xh(3,2:end));
for i=1:length(xh)
  text(xh(1,i), xh(2,i), xh(3,i), sprintf('%d', i));
end
xlabel('x'); ylabel('y'); zlabel('z');

figure(3);
title('camera pose');
plot3(campose(1,3:end), campose(2,3:end), campose(3,3:end));
for i=3:length(xh)
  text(campose(1,i), campose(2,i), campose(3,i), sprintf('%d', i));
end
xlabel('x'); ylabel('y'); zlabel('z');