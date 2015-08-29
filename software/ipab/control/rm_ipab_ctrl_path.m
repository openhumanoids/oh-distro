function rm_ipab_ctrl_path()

if ~exist('pods_get_base_path')
  % todo: implement the BUILD_PREFIX logic from the pod Makefiles (e.g.
  % search up to four directories higher)
  if ~exist('build/matlab')
    error('You must run make first (and/or add your pod build/matlab directory to the matlab path)');
  end
  addpath(fullfile(pwd,'build','matlab'));
end


rmpath_drake;

% remove the drake control matlab util directory into the matlab path:
rmpath(fullfile(pwd,'matlab'));
rmpath(fullfile(pwd,'matlab','test'));

end
