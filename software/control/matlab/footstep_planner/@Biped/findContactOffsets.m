function offset = findContactOffsets(biped)

	typecheck(biped,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});

	foot_body = struct('right', findLink(biped, biped.r_foot_name),...
	  'left', findLink(biped, biped.l_foot_name));

	foot_cen0 = struct();
  for f = {'right', 'left'}
    foot = f{1};
	  for g = {'toe', 'heel', 'inner'}
	    grp = g{1};
      gc = foot_body.(foot).contact_pts(:,...
        foot_body.(foot).collision_group{...
            cellfun(@(x) strcmp(x, grp), ...
             foot_body.(foot).collision_group_name)});
      offset.(foot).(grp) = mean(gc(1:3,:), 2);
    end
    offset.(foot).center = mean([offset.(foot).toe, offset.(foot).heel], 2);
  end
end
