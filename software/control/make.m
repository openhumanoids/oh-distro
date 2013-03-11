function make

% This file is intended to be run from the Makefile.  It assumes that the 
% environment variable 'BUILD_PREFIX' has been set.

p = cd('drake');
options.autoconfig=1;

cd(p);

%% write config/drc_control_setup.m

BUILD_PREFIX=getenv('BUILD_PREFIX');

fptr = fopen([BUILD_PREFIX,'/config/drc_control_setup.m'],'w');
if (fptr==-1)
  error('couldn''t open %s for writing\n',[BUILD_PREFIX,'/config/drc_control_setup.m']);
end

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
fprintf(fptr,'setenv(''ROS_PACKAGE_PATH'',''%s'');\n',getenv('ROS_PACKAGE_PATH'));

% path license
fprintf(fptr,'\n\n% Setup PATH LCP solver\n');
fprintf(fptr,'setenv(''PATH_LICENSE_STRING'',''2069810742&Courtesy_License&&&USR&2013&14_12_2011&1000&PATH&GEN&31_12_2013&0_0_0&0&0_0'');\n');

% add the drake control matlab util directory into the matlab path:
fprintf(fptr,'\n\n% Add drc control matlab utilities to the path\n');
fprintf(fptr,'addpath(''%s'');\n',fullfile(pwd,'matlab','util'));
fprintf(fptr,'addpath(''%s'');\n',fullfile(pwd,'collections_utils'));

fclose(fptr);

addpath([BUILD_PREFIX,'/config']);
drc_control_setup;

%% configure drake
configure(options);

%% build drake
make;


