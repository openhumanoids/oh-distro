function add_ctrl_path()

if ~isempty(getenv('PODS_MATLAB_SKIP_SETUP'))
  disp('Found PODS_MATLAB_SKIP_SETUP. Skipping addpath_control');
  return;
end

if ~exist('pods_get_base_path')
  % todo: implement the BUILD_PREFIX logic from the pod Makefiles (e.g.
  % search up to four directories higher)
  if ~exist('build/matlab')
    error('You must run make first (and/or add your pod build/matlab directory to the matlab path)');
  end
  addpath(fullfile(pwd,'build','matlab'));
end

% setup javaclasspath
javaaddpath([pods_get_base_path,'/share/java/lcmtypes_drc_lcmtypes.jar']);
javaaddpath([pods_get_base_path,'/share/java/lcmtypes_bot2-core.jar']);
javaaddpath([pods_get_base_path,'/share/java/lcmtypes_bot2-param.jar']);
javaaddpath([pods_get_base_path,'/share/java/drc_control.jar']);
javaaddpath([pods_get_base_path,'/share/java/lcmtypes_visualization.jar']);
javaaddpath([pods_get_base_path,'/share/java/lcmtypes_multisense.jar']);
javaaddpath([pods_get_base_path,'/share/java/lcmtypes_robotiqhand.jar']);
javaaddpath([pods_get_base_path,'/share/java/lcmtypes_microstrain.jar']);
% javaaddpath([pods_get_base_path,'/share/java/lcmtypes_scanmatch.jar']);
javaaddpath([pods_get_base_path,'/share/java/bot2-lcmgl.jar']);

% tell drake about ROS
setenv('DRC_PATH',[pods_get_base_path,'/..']);
% Removed Nov 2015 - previous referred to mit_drcsim_scripts
%[~,ROS_MODEL_PKG] = system(['grep ROS_MODEL_PKG pod-build/CMakeCache.txt | cut -d "=" -f2']);
%ROS_MODEL_PKG = strtrim(ROS_MODEL_PKG);
%setenv('ROS_PACKAGE_PATH',[ROS_MODEL_PKG,pathsep,getenv('ROS_PACKAGE_PATH')]);

setenv('LD_LIBRARY_PATH',[getenv('LD_LIBRARY_PATH'),pathsep,'/opt/ros/fuerte/lib']);

% path license
setenv('PATH_LICENSE_STRING','2069810742&Courtesy_License&&&USR&2013&14_12_2011&1000&PATH&GEN&31_12_2013&0_0_0&0&0_0');

addpath_drake;

checkDependency('lcm');
checkDependency('lcmgl');
checkDependency('gurobi');
checkDependency('snopt');
addpath_mosek;

addpath([pods_get_base_path,'/matlab']);

% add the drake control matlab util directory into the matlab path:
addpath(fullfile(pwd,'..','drake/drake','examples','ZMP'));
addpath(fullfile(pwd,'..','drake/drake','examples','Atlas'));
addpath(fullfile(pwd,'..','drake/drake','examples','Valkyrie'));
addpath(fullfile(pwd,'matlab'));
addpath(fullfile(pwd,'matlab','calibration'));
addpath(fullfile(pwd,'matlab','controllers'));
addpath(fullfile(pwd,'matlab','planners'));
addpath(fullfile(pwd,'matlab','planners','constraints'));
addpath(fullfile(pwd,'matlab','planners','drill_planner'));
addpath(fullfile(pwd,'matlab','planners','ladder_planner'));
addpath(fullfile(pwd,'matlab','planners','driving_planner'));
addpath(fullfile(pwd,'matlab','planners','footstep_planner'));
addpath(fullfile(pwd,'matlab','planners','walking_planner'));
addpath(fullfile(pwd,'matlab','planners','atlas_keyframe_planners'));
addpath(fullfile(pwd,'matlab', 'planners', 'collision_free_solver'));
addpath(fullfile(pwd,'matlab','util'));
addpath(fullfile(pwd,'matlab','test'));
addpath(fullfile(pwd,'matlab','systems'));
addpath(fullfile(pwd,'collections_utils'));

end
