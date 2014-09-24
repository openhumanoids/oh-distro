function pose_indices = findCalibrationPoseIndices(v_data, num_poses, v_norm_limit)
% FINDPOSEINDICES Tries to find a given number of poses that can be used for
% calibration purposes.
%
% @param v_data velocity data (columns are velocity vectors at different times)
% @param num_poses number of poses to find
% @param v_norm_limit norm of velocity state vector below which robot is
% considered stationary.
% @retval pose_indices state data matrix column indices in the center of
% regions where the robot's velocity is low.

v_norm = sum(sqrt(v_data .* v_data), 1);

indices = v_norm < v_norm_limit;

previous_beneath_limit = false;
starting_indices = [];
counts = [];
for i = 1 : length(indices)
  if indices(i)
    if previous_beneath_limit
      counts(end) = counts(end) + 1;
    else
      starting_indices(end + 1) = i; %#ok<AGROW>
      counts(end + 1) = 1; %#ok<AGROW>
    end
    previous_beneath_limit = true;
  else
    previous_beneath_limit = false;
  end
end

if length(starting_indices) < num_poses
  warning(['Could only find ' num2str(length(starting_indices)) ' poses.']);
  num_poses = length(starting_indices);
end

[sorted_counts, sorting_indices] = sort(counts, 'descend');
max_counts = sorted_counts(1 : num_poses);
max_count_indices = sorting_indices(1 : num_poses);
max_count_starting_indices = starting_indices(max_count_indices);
pose_indices = max_count_starting_indices + floor(max_counts / 2); % center
pose_indices = sort(pose_indices);
end

