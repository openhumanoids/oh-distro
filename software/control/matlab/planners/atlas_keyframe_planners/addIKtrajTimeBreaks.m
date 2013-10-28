function t_breaks = addIKtrajTimeBreaks(iktraj_tbreaks,constraint_cell)
  % A utility function to take all boundary times of each constraint
  % in the constraint cell, append them to iktraj_tbreaks, and return
  % the unique time points of the appended array.
  % @param iktraj_tbreaks     An array or original time breaks for ik
  %                           traj
  % @param constraint_cell    A cell containing constraints
  % @param t_breaks           A sorted array containing all unique
  %                           moments being the boundary of time
  %                           spans of each constraint, together with
  %                           the original iktraj_tbreaks
  tmax = max(iktraj_tbreaks);
  tmin = min(iktraj_tbreaks);
  t_breaks = iktraj_tbreaks;
  for constraint_idx = 1:length(constraint_cell)
    tspan = constraint_cell{constraint_idx}.getTspan();
    tspan = tspan(tspan<=tmax & tspan >=tmin);
    t_breaks = [t_breaks tspan];
  end
  t_breaks_tmp = sort(t_breaks);
  t_breaks  = t_breaks_tmp(1);
  for i = 2:length(t_breaks_tmp)
    if(t_breaks_tmp(i)-t_breaks_tmp(i-1)>1e-3)
      t_breaks = [t_breaks t_breaks_tmp(i)];
    end
  end
  
end