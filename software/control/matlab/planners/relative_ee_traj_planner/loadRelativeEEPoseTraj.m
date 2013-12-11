function [ee_pose_relative,T] = loadRelativeEEPoseTraj(traj_name,is_left_sided)
    traj_dir = [getenv('DRC_BASE') '/software/control/matlab/planners/relative_ee_traj_planner'];
    logical2LeftRight = @(b) regexprep(sprintf('%i',b),{'1','0'},{'LEFT','RIGHT'});
    filename = fullfile(traj_dir,[traj_name '_' logical2LeftRight(is_left_sided) '.mat']);
    filename_flipped = fullfile(traj_dir,[traj_name '_' logical2LeftRight(~is_left_sided) '.mat']);
    flip_traj = false;
    if exist(filename,'file')
      S = load(filename);
    elseif exist(filename_flipped,'file')
      flip_traj = true;
      S = load(filename_flipped);
    else
      error('loadRelativeEEPoseTraj:file_not_found',...
        'Could not find a saved trajecory matching ''%s''', ...
        filename);
    end
    T = S.T;
    ee_pose_relative = S.ee_pose_relative;
    nt = size(ee_pose_relative,2);
    if flip_traj
      ee_pose_relative(2,:) = -ee_pose_relative(2,:);
      for i=1:nt, ee_pose_relative(4:7,i) = quatConjugate(ee_pose_relative(4:7,i)); end;
    end
end
