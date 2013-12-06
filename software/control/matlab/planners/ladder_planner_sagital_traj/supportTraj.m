function traj = supportTraj(body_idx,support_times,support)
  support_data = zeros(size(support_times));
  support_data(1:end) = ...
    cellfun(@(supp) double(any(supp.bodies==body_idx)),support); 
  support_data(end) = support_data(end-1);
  traj = PPTrajectory(zoh(support_times,support_data));
end
