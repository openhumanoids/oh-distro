function rmpath_control()

if ~exist('pods_get_base_path')
  % todo: implement the BUILD_PREFIX logic from the pod Makefiles (e.g.
  % search up to four directories higher)
  if ~exist('build/matlab')
    error('You must run make first (and/or add your pod build/matlab directory to the matlab path)');
  end
  addpath(fullfile(pwd,'build','matlab'));  
end

% setup javaclasspath
javarmpath([pods_get_base_path,'/share/java/lcmtypes_drc_lcmtypes.jar']);
javarmpath([pods_get_base_path,'/share/java/lcmtypes_bot2-core.jar']);
javarmpath([pods_get_base_path,'/share/java/drc_control.jar']);
javarmpath([pods_get_base_path,'/share/java/lcmtypes_visualization.jar']);
javarmpath([pods_get_base_path,'/share/java/lcmtypes_scanmatch.jar']);

% add the drake control matlab util directory into the matlab path:
rmpath(fullfile(pwd,'matlab'));
rmpath(fullfile(pwd,'matlab','controllers'));
rmpath(fullfile(pwd,'matlab','planners'));
rmpath(fullfile(pwd,'matlab','planners','footstep_planner'));
rmpath(fullfile(pwd,'matlab','planners','pinned_manipulation','spherical_interp'));
rmpath(fullfile(pwd,'matlab','util'));
rmpath(fullfile(pwd,'matlab','frames'));
rmpath(fullfile(pwd,'matlab','test'));
rmpath(fullfile(pwd,'matlab','systems'));
rmpath(fullfile(pwd,'collections_utils'));

end
