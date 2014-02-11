function [x_traj,info] = interpolatingTraj(r,tspan,xstar,x0,percent_still_time,constraints,ikoptions)
  t0 = tspan(1);
  tf = tspan(end);
  duration = tf-t0;
  still_duration = duration*percent_still_time/(length(xstar)+2);
  move_duration = duration*(1-percent_still_time)/(length(xstar)+1);
  t = t0:(still_duration+move_duration):tf;
  t = unique([t,(t0+still_duration):(move_duration+still_duration):tf]);
  t_sparse = t;
  n_move_pts = 5;
  for i = 1:n_move_pts
    t = unique([t,(t0+still_duration+i*move_duration/n_move_pts):(move_duration+still_duration):tf]);
  end
  x_data = [x0,cell2mat(cellfun(@double,xstar,'UniformOutput',false)),x0];
  n = size(x_data,2);
  x_data = reshape(repmat(x_data,2,1),r.getNumStates(),2*n);
  x_traj = PPTrajectory(foh(t_sparse,x_data));
  q_nom_traj = x_traj(1:r.getNumDOF());
  q_nom = squeeze(eval(q_nom_traj,t));
  [q_data,info] = inverseKinPointwise(r,t,q_nom,q_nom,constraints{:},ikoptions);
  idx_good = info < 10;
  for i = find(~idx_good)
    if isa(constraints{2},'AllBodiesClosestDistanceConstraint') && constraints{2}.checkConstraint(q_data(:,i))
      idx_good(i) = true;
      info(i) = 5;
    end
  end
  x_traj = PPTrajectory(foh(t(idx_good),[q_data(:,idx_good);q_data(:,idx_good)*0]));
  x_traj = x_traj.setOutputFrame(r.getStateFrame());
end
