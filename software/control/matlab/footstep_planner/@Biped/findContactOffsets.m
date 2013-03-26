function offset = findContactOffsets(biped)

	typecheck(biped,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});

	foot_body = struct('right', findLink(biped, biped.r_foot_name),...
	  'left', findLink(biped, biped.l_foot_name));

	foot_cen0 = struct();
  for f = {'right', 'left'}
    foot = f{1};
	  gc = foot_body.(foot).contact_pts;
	  k = convhull(gc(1:2,:)');
	  offset.(foot).center = mean(gc(1:3, k),2);
	  for g = {'toe', 'heel'}
	    grp = g{1};
      gc = foot_body.(foot).contact_pts(:,...
        foot_body.(foot).collision_group{...
            cellfun(@(x) strcmp(x, grp), ...
             foot_body.(foot).collision_group_name)});
      offset.(foot).(grp) = mean(gc(1:3,:), 2);
    end
  end
end
