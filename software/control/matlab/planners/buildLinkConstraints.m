function link_constraints = buildLinkConstraints(biped, q0, foottraj, fixed_links)

kinsol = doKinematics(biped,q0);

%% Convert the foottraj to an easier format to hand to IK
link_constraints(1) = struct('link_ndx', biped.findLinkInd(biped.foot_bodies.right.linkname), 'pt', [0;0;0], 'min_traj', [], 'max_traj', [], 'traj', foottraj.right.orig);
link_constraints(2) = struct('link_ndx', biped.findLinkInd(biped.foot_bodies.left.linkname), 'pt', [0;0;0], 'min_traj', [], 'max_traj', [], 'traj', foottraj.left.orig);
for f = {'right', 'left'}
  foot = f{1};
  for g = {'toe', 'heel'}
    grp = g{1};
    for pt_ndx = biped.foot_bodies.(foot).collision_group{strcmp(biped.foot_bodies.(foot).collision_group_name, grp)}
      if (~isempty(foottraj.(foot).(grp).min) && ~isempty(foottraj.(foot).(grp).max))
        link_constraints(end+1) = struct('link_ndx', biped.findLinkInd(biped.foot_bodies.(foot).linkname), 'pt', biped.foot_bodies.(foot).contact_pts(:,pt_ndx), 'min_traj', foottraj.(foot).(grp).min, 'max_traj', foottraj.(foot).(grp).max, 'traj', []);
      end
    end
  end
end


%% Allow the user to fix the current position of some links (useful for walking while holding the hand still, e.g.)
for j = 1:length(fixed_links)
  if isa(fixed_links(j).link, 'RigidBody')
    link_ndx = find(strcmp(biped.getLinkNames(), fixed_links(j).link.linkname),1);
  else
    link_ndx = fixed_links(j);
  end
  pos = biped.forwardKin(kinsol, link_ndx, fixed_links(j).pt,0);
  pos_min = pos - fixed_links(j).tolerance;
  pos_max = pos + fixed_links(j).tolerance;
  link_constraints(end+1) = struct('link_ndx', link_ndx, 'pt',fixed_links(j).pt, 'min_traj', ConstantTrajectory(pos_min), 'max_traj', ConstantTrajectory(pos_max), 'traj', []);
end

