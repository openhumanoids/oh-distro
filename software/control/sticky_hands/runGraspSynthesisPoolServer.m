function runGraspSynthesisPoolServer(n_workers)
%NOTEST
mode = 1; % 0 = robot, 1 = base
if mode ==1
  lcm_url = 'udpm://239.255.12.68:1268?ttl=1';
else
  lcm_url = 'udpm://239.255.76.67:7667?ttl=1';
end

lcm_url = 'udpm://239.255.76.67:7667?ttl=0';
%lcm.lcm.LCM.getSingletonTemp(lcm_url); % only works on mfallons machine

%profile on;
no_of_workers = n_workers;
if(matlabpool('size')==0)
matlabpool(no_of_workers)
end
%
spmd
    runGraspOptSinglePoolService(no_of_workers);    
end % end spmd

% end
matlabpool close
%profile viewer;
end

% How to get rid SPMD overhead at startup?
