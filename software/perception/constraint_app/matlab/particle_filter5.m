function [xhk, pf] = particle_filter5(yk, pf, resampling_strategy)
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


    %% Initialize variables
    Ns = pf.Ns;                              % number of particles

    if pf.k == 0
       pf.xkm1 = pf.gen_x0(pf.x0, pf.Ns, pf.task);
       wkm1 = repmat(1/Ns, Ns, 1);           % all particles have the same weight
       pf.resample_count = 0;
    else
        wkm1 = pf.wkm1;                      % weights of last iteration
    end

    %% Separate memory
    xkm1 = pf.xkm1; % extract particles from last iteration;

    [xk, weight_scales] = pf.importance_sampling(xkm1, yk, pf);
    wk = wkm1 .* weight_scales;

    pf.last_xk = xk;
    pf.last_wk_before_normalization = wk;

    if any(isnan(wk))
        error('got nan while calculating wk');
    end

    %% Normalize weight vectors
    wk = wk./sum(wk);

    %% Calculate effective sample size: eq 48, Ref 1
    pf.neff = 1/sum(wk.^2);

    %% Resampling
    % remove this condition and sample on each iteration:
    % [xk, wk] = resample(xk, wk, resampling_strategy);
    %if you want to implement the bootstrap particle filter
    resample_percentage = 0.50;
    Nt = resample_percentage*Ns;
    fprintf('Neff = %f of Nt = %f\n', pf.neff, Nt);
    if isnan(pf.neff)
        error('Neff became nan');
    end
    if pf.neff < Nt
       disp('Resampling ...')
       [xk, wk, idx] = resample(xk, wk, resampling_strategy);
       pf.resample_count = pf.resample_count + 1;
       % {xk, wk} is an approximate discrete representation of p(x_k | y_{1:k})
    end

    %% Compute estimated state
    % xhk = zeros(nx,1);
    % if any(any((xk(3:end,:)>3),2) & any((xk(3:end,:)<-3),2))
    %     error('some of the angles in the state are wrapping at +/- pi');
    % end
    % for i = 1:Ns;
    %    xhk = xhk + wk(i)*xk(:,i);
    % end
    %xhk = weightedSE2Mean(xk, wk', [3 4 5 6]);
    xhk = pf.weightedMean(xk, wk);

    %% Store new weights and particles
    pf.wkm1 = wk;
    pf.xkm1 = xk;
    pf.k    = pf.k + 1;


function [xk, wk, idx] = resample(xk, wk, resampling_strategy)
    % Resampling function

    Ns = length(wk);  % Ns = number of particles


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
       otherwise
          error('Resampling strategy not implemented')
    end;

    xk = xk(:,idx);                    % extract new particles
    wk = repmat(1/Ns, Ns, 1);          % now all particles have the same weight
