function [qtraj,breaks] = constructAndRescaleTrajectory(q_vals,qd_max,add_pauses)
  if nargin < 3
    add_pauses = 0;
  end
  N = size(q_vals,2);
  if add_pauses
    qtraj_cell = cell(N,1);
    for j = 1:N-1
      qtraj_cell{j} = PPTrajectory(pchip([1,2],q_vals(:,j:j+1)));
      qtraj_cell{j} = rescalePlanTiming(qtraj_cell{j},qd_max);
    end
    qtraj = qtraj_cell{1};
    breaks = qtraj.tspan();

    for j = 2:N
      qtraj_cell{j} = qtraj_cell{j}.shiftTime(qtraj.tspan(2));
      qtraj = qtraj.append(qtraj_cell{j});
      breaks(end+1) = qtraj.tspan(2);
    end
  else
    qtraj = PPTrajectory(pchip([1:N],q_vals));
    qtraj = rescalePlanTiming(qtraj,qd_max);
    breaks = qtraj.tspan();
  end
end