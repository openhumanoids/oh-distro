function [qtraj_rescaled, traj_breaks_rescaled] = concatAndRescaleTrajectories(qtraj_cell, qd_max, varargin)
    qtraj = qtraj_cell{1};
    traj_breaks = qtraj.tspan();

    for j = 2:numel(qtraj_cell)
      qtraj_cell{j} = qtraj_cell{j}.shiftTime(qtraj.tspan(2));
      qtraj = qtraj.append(qtraj_cell{j});
      traj_breaks(end+1) = qtraj.tspan(2);
    end

    [qtraj_rescaled, traj_breaks_rescaled] = rescalePlanTimingGetOriginalTimes(qtraj,qd_max, traj_breaks, varargin{:});
    traj_breaks_rescaled = traj_breaks_rescaled';
end
