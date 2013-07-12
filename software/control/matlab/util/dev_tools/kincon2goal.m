function goal = kincon2goal(kincon)
  goal = drc.contact_goal_t();
  goal.object_1_name = kincon.robot.body(kincon.body_ind).linkname;
  goal.object_1_contact_grp = kincon.contact_grp;
  goal.object_2_name = '';
  goal.object_2_contact_grp = '';

  goal.lower_bound_completion_time = kincon.tspan(1);
  goal.upper_bound_completion_time = kincon.tspan(2);
  pos.min = kincon.pos_min.pt(:,1);
  pos.max = kincon.pos_max.pt(:,1);
  pos_diff = pos.max - pos.min;
  if isinf(pos.min(1))
    if isinf(pos.max(1))
      goal.x_relation = goal.UNDEFINED;
    else
      goal.x_relation = goal.REL_LESS_THAN;
    end
  elseif isinf(pos.max(1))
    goal.x_relation = goal.REL_GREATER_THAN;
  else
    goal.x_relation = goal.REL_EQUAL;
  end
  if isinf(pos.min(2))
    if isinf(pos.max(2))
      goal.y_relation = goal.UNDEFINED;
    else
      goal.y_relation = goal.REL_LESS_THAN;
    end
  elseif isinf(pos.max(2))
    goal.y_relation = goal.REL_GREATER_THAN;
  else
    goal.y_relation = goal.REL_EQUAL;
  end
  if isinf(pos.min(3))
    if isinf(pos.max(3))
      goal.z_relation = goal.UNDEFINED;
    else
      goal.z_relation = goal.REL_LESS_THAN;
    end
  elseif isinf(pos.max(3))
    goal.z_relation = goal.REL_GREATER_THAN;
  else
    goal.z_relation = goal.REL_EQUAL;
  end

  switch goal.x_relation
    case 0
      p(1) = pos.min(1);
    case 1
      p(1) = pos.max(1);
    case 2
      p(1) = pos.min(1);
    case 3
      p(1) = 0;
  end
  switch goal.y_relation
    case 0
      p(2) = pos.min(2);
    case 1
      p(2) = pos.max(2);
    case 2
      p(2) = pos.min(2);
    case 3
      p(2) = 0;
  end
  switch goal.z_relation
    case 0
      p(3) = pos.min(3);
    case 1
      p(3) = pos.max(3);
    case 2
      p(3) = pos.min(3);
    case 3
      p(3) = 0;
  end

  goal.target_pt = drc.vector_3d_t();
  goal.target_pt.x = p(1);
  goal.target_pt.y = p(2);
  goal.target_pt.z = p(3);

  % This should be changed as support for other contact points is added
  % contact_state = -1 undefined
  % contact_state = 0 not in contact
  % contact_state = 1 makes contact right at that moment
  % contact_state = 2 breaks contact right at that moment
  % contact_state = 3 in static planar contact
  % contact_state = 4 in static gripping contact
  if(all(kincon.contact_statei{1} == kincon.NOT_IN_CONTACT))
    goal.contact_type = goal.NOT_IN_CONTACT;
  elseif(all(kincon.contact_statei{1} == kincon.COLLISION_AVOIDANCE))
    goal.contact_type = goal.COLLISION_AVOIDANCE;
  else
    goal.contact_type = goal.ON_GROUND_PLANE;
  end
end
