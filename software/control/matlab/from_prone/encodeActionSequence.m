function action_sequence = encodeActionSequence(robot,fdata)

tspan_start = 0;
action_sequence = ActionSequence();
for i= 1:fdata.num_contact_goals
    body = findLink(robot,fdata.object_1_name(i));
    body_pts = body.getContactPoints(char(fdata.object_1_grp(i)));
    if(fdata.target_pts_relation(i) == 0) % equal to
        worldpos = fdata.target_pts(:,i);
    elseif(fdata.target_pts_relation(i) == 1) % less than
        worldpos = struct();
        worldpos.max = fdata.target_pts(:,i);
        worldpos.min = -inf(size(worldpos.max));
    elseif(fdata.target_pts_relation(i) == 2) % greater than
        worldpos = struct();
        worldpos.min = fdata.target_pts(:,i);
        worldpos.max = inf(size(worldpos.min));
    end
    tspan = [tspan_start tspan_start + (fdata.lb_completion_time(i)+fdata.ub_completion_time(i))/2]; % This is a purely hack, since I do not know how to handle variable time for kinematic constraints
    tspan_start = tspan(end);
    name = body.linkname;
    action_constraint = ActionKinematicConstraint(body,body_pts,worldpos,tspan,name);
    action_sequence = action_sequence.addKinematicConstraint(action_constraint);
end

end