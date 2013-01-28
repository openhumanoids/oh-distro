function runGraspSynthesis4PoolServer()
no_of_workers =  4;
matlabpool(no_of_workers) 
%
spmd
    runGraspOptSinglePoolService(no_of_workers);    
end % end spmd

% end
matlabpool close

end


% How to get rid SPMD overhead at startup?