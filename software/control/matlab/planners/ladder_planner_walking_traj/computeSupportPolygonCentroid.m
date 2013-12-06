function centroid = computeSupportPolygonCentroid(t,feet)
  foot_pts_in_world = [];
  for i = 1:length(feet)
    if feet(i).support_traj.eval(t-0.1) && feet(i).support_traj.eval(t+0.1)
      foot_pts = feet(i).support_pts;
      foot_pos = feet(i).traj.eval(t);
      T_foot_to_world = [rpy2rotmat(foot_pos(4:6)),foot_pos(1:3); ...
        zeros(1,3),1];
      curr_foot_pts_in_world = T_foot_to_world*[foot_pts;ones(1,size(foot_pts,2))];
      foot_pts_in_world = [foot_pts_in_world, curr_foot_pts_in_world(1:3,:)];
      centroid = mean(foot_pts_in_world,2);
    end
  end
end