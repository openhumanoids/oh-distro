function [xhk, pf] = particle_filter(sys, yk, pf, resampling_strategy, camposek)
%% Generic particle filter
%
% Note: when resampling is performed on each step this algorithm is called
% the Bootstrap particle filter
%
% Usage:
% [xhk, pf] = particle_filter(sys, yk, pf, resamping_strategy)
%
% Inputs:
% sys  = function handle to process equation
% yk   = observation vector at time k (column vector)
% pf   = structure with the following fields
%   .k                = iteration number
%   .Ns               = number of particles
%   .w                = weights   (Ns x T)
%   .particles        = particles (nx x Ns x T)
%   .gen_x0           = function handle of a procedure that samples from the initial pdf p_x0
%   .p_yk_given_xk    = function handle of the observation likelihood PDF p(y[k] | x[k])
%   .gen_sys_noise    = function handle of a procedure that generates system noise
% resampling_strategy = resampling strategy. Set it either to 
%                       'multinomial_resampling' or 'systematic_resampling'
%
% Outputs:
% xhk   = estimated state
% pf    = the same structure as in the input but updated at iteration k
%
% Reference:
% [1] Arulampalam et. al. (2002).  A tutorial on particle filters for 
%     online nonlinear/non-gaussian bayesian tracking. IEEE Transactions on 
%     Signal Processing. 50 (2). p 174--188

%% Programmed by:
% Diego Andres Alvarez Marin (diegotorquemada@gmail.com)
% Universidad Nacional de Colombia at Manizales, February 29, 2012

%%
k = pf.k;
if k == 1
   error('error: k must be an integer greater or equal than 2');
end

%% Initialize variables
Ns = pf.Ns;                              % number of particles
nx = size(pf.particles,1);               % number of states

wkm1 = pf.w(:, k-1);                     % weights of last iteration
if k == 2
   for i = 1:Ns                          % simulate initial particles
      pf.particles(:,i,1) = pf.gen_x0(camposek); % at time k=1
   end   
   wkm1 = repmat(1/Ns, Ns, 1);           % all particles have the same weight
end

%%
% The importance sampling function:
% PRIOR: (this method is sensitive to outliers)   THIS IS THE ONE USED HERE
% q_xk_given_xkm1_yk = pf.p_xk_given_xkm1;

% OPTIMAL:
% q_xk_given_xkm1_yk = q_xk_given_xkm1^i_yk;
% Note this PDF can be approximated by MCMC methods: they are expensive but 
% they may be useful when non-iterative schemes fail

%% Separate memory
xkm1 = pf.particles(:,:,k-1); % extract particles from last iteration;
xk   = zeros(size(xkm1));     % = zeros(nx,Ns);
wk   = zeros(size(wkm1));     % = zeros(Ns,1);

%% Algorithm 3 of Ref [1]
for i = 1:Ns
   % xk(:,i) = sample_vector_from q_xk_given_xkm1_yk given xkm1(:,i) and yk
   % Using the PRIOR PDF: pf.p_xk_given_xkm1: eq 62, Ref 1.
   xk(:,i) = sys(k, xkm1(:,i), pf.gen_sys_noise());
   
   % Equation 48, Ref 1.
   % wk(i) = wkm1(i) * p_yk_given_xk(yk, xk(:,i))*p_xk_given_xkm1(xk(:,i), xkm1(:,i))/q_xk_given_xkm1_yk(xk(:,i), xkm1(:,i), yk);
   
   % weights (when using the PRIOR pdf): eq 63, Ref 1
   wk(i) = wkm1(i) * pf.p_yk_given_xk(k, yk, xk(:,i), camposek);
   
   % weights (when using the OPTIMAL pdf): eq 53, Ref 1
   % wk(i) = wkm1(i) * p_yk_given_xkm1(yk, xkm1(:,i)); % we do not know this PDF
end;

%% Normalize weight vector
wk = wk./sum(wk);

%%hack
% figure(4)
% scatter3(xk(1,:),xk(2,:),xk(3,:),3,wk);
% xlabel('x'); ylabel('y'); zlabel('z');
% axis([0 1 -1 1 -1 1 0 1])
% axis equal
% view(-90,90)
% 
% 
% lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'pf');
% lcmgl.glColor3f(1,0,1);
% for i=1:length(xk)
%   lcmgl.sphere(xk(1:3, i),0.01,100,100);
% end
% lcmgl.switchBuffers;

%% Calculate effective sample size: eq 48, Ref 1
Neff = 1/sum(wk.^2);

%% Resampling
% remove this condition and sample on each iteration:
% [xk, wk] = resample(xk, wk, resampling_strategy);
%if you want to implement the bootstrap particle filter
resample_percentaje = 0.50;
Nt = resample_percentaje*Ns;
if Neff < Nt
   disp('Resampling ...')
   [xk, wk] = resample(xk, wk, resampling_strategy);
   % {xk, wk} is an approximate discrete representation of p(x_k | y_{1:k})
end

%% Compute estimated state
xhk = zeros(nx,1);
for i = 1:Ns;
   xhk = xhk + wk(i)*xk(:,i);
end

%% Store new weights and particles
pf.w(:,k) = wk;
pf.particles(:,:,k) = xk;

return; % bye, bye!!!

%% Resampling function
function [xk, wk, idx] = resample(xk, wk, resampling_strategy)

Ns = length(wk);  % Ns = number of particles

% wk = wk./sum(wk); % normalize weight vector (already done)

switch resampling_strategy
   case 'multinomial_resampling'
      with_replacement = true;
      idx = randsample(1:Ns, Ns, with_replacement, wk);
%{
      THIS IS EQUIVALENT TO:
      edges = min([0 cumsum(wk)'],1); % protect against accumulated round-off
      edges(end) = 1;                 % get the upper edge exact
      % this works like the inverse of the empirical distribution and returns
      % the interval where the sample is to be found
      [~, idx] = histc(sort(rand(Ns,1)), edges);
%}
   case 'systematic_resampling'
      % this is performing latin hypercube sampling on wk
      edges = min([0 cumsum(wk)'],1); % protect against accumulated round-off
      edges(end) = 1;                 % get the upper edge exact
      u1 = rand/Ns;
      % this works like the inverse of the empirical distribution and returns
      % the interval where the sample is to be found
      [~, idx] = histc(u1:1/Ns:1, edges);
   % case 'regularized_pf'      TO BE IMPLEMENTED
   % case 'stratified_sampling' TO BE IMPLEMENTED
   % case 'residual_sampling'   TO BE IMPLEMENTED
   otherwise
      error('Resampling strategy not implemented')
end;

xk = xk(:,idx);                    % extract new particles
wk = repmat(1/Ns, 1, Ns);          % now all particles have the same weight

return;  % bye, bye!!!
