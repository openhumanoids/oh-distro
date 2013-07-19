function write_drc_control_setup

% This file is intended to be run from the Makefile.  It assumes that the 
% environment variable 'BUILD_PREFIX' has been set.

%% write config/drc_control_setup.m

BUILD_PREFIX=getenv('BUILD_PREFIX');

fptr = fopen([BUILD_PREFIX,'/config/drc_control_setup.m'],'w');
if (fptr==-1)
  error('couldn''t open %s for writing\n',[BUILD_PREFIX,'/config/drc_control_setup.m']);
end

% call pathdef (can't believe that i have to do this!)
% fprintf(fptr,'\n\npath(pathdef);\n');

fprintf(fptr,'addpath_drake;\n');
fprintf(fptr,'addpath_eigen_utils;\n');
fprintf(fptr,'addpath_gurobi;\n');
fprintf(fptr,'addpath_matlab_utils;\n');
fprintf(fptr,'addpath_snopt;\n');
fprintf(fptr,'addpath_spotless;\n');
fprintf(fptr,'addpath_control;\n');

fclose(fptr);

end



