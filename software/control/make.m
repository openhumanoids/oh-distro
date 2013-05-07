function make

% This file is intended to be run from the Makefile.  It assumes that the 
% environment variable 'BUILD_PREFIX' has been set.

%% write config/drc_control_setup.m

BUILD_PREFIX=getenv('BUILD_PREFIX');

fptr = fopen([BUILD_PREFIX,'/config/drc_control_setup.m'],'w');
if (fptr==-1)
  error('couldn''t open %s for writing\n',[BUILD_PREFIX,'/config/drc_control_setup.m']);
end

% call pathdef (can't believe that i have to do this!)
fprintf(fptr,'\n\npath(pathdef);\n');

% setup javaclasspath
fprintf(fptr,'\n\n% Setup javaclasspath\n');
[~,lcmclasspath]=system('pkg-config lcm-java --variable=classpath');
fprintf(fptr,'javaaddpath(''%s'');\n',strtrim(lcmclasspath));
fprintf(fptr,'javaaddpath(''%s/share/java/lcmtypes_drc_lcmtypes.jar'');\n',BUILD_PREFIX);
fprintf(fptr,'javaaddpath(''%s/drake/drake.jar'');\n',pwd);
fprintf(fptr,'javaaddpath(''%s/share/java/drc_control.jar'');\n',BUILD_PREFIX);
fprintf(fptr,'javaaddpath(''%s/share/java/lcmtypes_visualization.jar'');\n',BUILD_PREFIX);

% tell drake about ROS
fprintf(fptr,'\n\n% Set environment variables\n');
fprintf(fptr,'setenv(''DRC_PATH'',''%s'');\n',fullfile(pwd,'..'));
fprintf(fptr,'setenv(''ROS_ROOT'',''%s'');\n',getenv('ROS_ROOT'));
fprintf(fptr,'setenv(''ROS_PACKAGE_PATH'',''%s'');\n',[fullfile(BUILD_PREFIX,'..','..','ros_workspace'),pathsep,getenv('ROS_PACKAGE_PATH')]);

% path license
fprintf(fptr,'\n\n% Setup PATH LCP solver\n');
fprintf(fptr,'setenv(''PATH_LICENSE_STRING'',''2069810742&Courtesy_License&&&USR&2013&14_12_2011&1000&PATH&GEN&31_12_2013&0_0_0&0&0_0'');\n');

% add the drake control matlab util directory into the matlab path:
fprintf(fptr,'\n\n% Add drc control matlab utilities to the path\n');
fprintf(fptr,'addpath(''%s'');\n',fullfile(pwd,'matlab'));
fprintf(fptr,'addpath(''%s'');\n',fullfile(pwd,'matlab','util'));
fprintf(fptr,'addpath(''%s'');\n',fullfile(pwd,'matlab','frames'));
fprintf(fptr,'addpath(''%s'');\n',fullfile(pwd,'matlab','footstep_planner'));
fprintf(fptr,'addpath(''%s'');\n',fullfile(pwd,'matlab','pinned_manipulation/spherical_interp'));
fprintf(fptr,'addpath(''%s'');\n',fullfile(pwd,'collections_utils'));
fprintf(fptr,'addpath(''%s'');\n',fullfile(BUILD_PREFIX,'matlab'));

fclose(fptr);

run([BUILD_PREFIX,'/config/drc_control_setup']);

p = cd('drake');
options.autoconfig=1;
system(['touch ',fullfile(strtok(userpath,':'),'pathdef.m')]);

%% configure drake
configure(options);

%% build drake
make;

cd(p);

%% build drc drake mexfiles

if (0) % build maps
  [~,cflags]=system('pkg-config --cflags maps eigen3 lcm');
  incs = regexp(cflags,'-I\S+','match'); incs = sprintf('%s ',incs{:});
  
  [~,libs]=system('pkg-config --libs maps eigen3 lcm');
  libs = strrep(libs,'-pthread','');%-lpthread');
  
  cmdstr = ['mex -v src/mapAPIwrapper.cpp -O -outdir ',BUILD_PREFIX,'/matlab ',incs,' ',libs];
  %cmdstr = ['mex -v src/mapAPIwrapper.cpp LDFLAGS=''\$LDFLAGS -static-libstdc++'' -O -outdir ',BUILD_PREFIX,'/matlab ',incs,' ',libs];
  disp(cmdstr);
  eval(cmdstr);
  
  cmdstr = ['mex -v src/MapWrapperRobot.cpp -O -outdir ',BUILD_PREFIX,'/matlab ',incs,' ',libs];
  disp(cmdstr);
  eval(cmdstr);
  
  cmdstr = ['mex -v src/bot_timestamp_now.cpp -O -outdir ',BUILD_PREFIX,'/matlab ',incs,' ',libs];
  disp(cmdstr);
  eval(cmdstr);
end

if strcmpi(getenv('USER'),'russt') && ismac   % sorry... :)
  load drake_config;
  gurobi_os_dir = '/Library/gurobi510/mac64';
  args = {'-largeArrayDims', ...
    ['-I',conf.eigen3_incdir], ...
    fullfile(getDrakePath(),'systems','plants',['RigidBodyManipulator.',objext]), ...
    ['-I',fullfile(getDrakePath(),'systems','plants')], ...
    ['-I',fullfile(gurobi_os_dir,'/include')], ...
    ['-L',fullfile(gurobi_os_dir,'lib')],...
    '-lgurobi51'};
  cmdstr = ['mex -g src/QPControllermex.cpp -outdir ',BUILD_PREFIX,'/matlab ',sprintf('%s ',args{:})];
  disp(cmdstr);
  eval(cmdstr);
end

end



