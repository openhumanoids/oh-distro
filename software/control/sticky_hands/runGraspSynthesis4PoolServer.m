function runGraspSynthesis4PoolServer()
%profile on;
no_of_workers =  2;
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