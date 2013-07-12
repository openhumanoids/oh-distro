function offset = findContactOffsets(biped)

	typecheck(biped,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});

  for f = {'right', 'left'}
    foot = f{1};
	  for g = {'toe', 'heel', 'inner'}
	    grp = g{1};
      gc = biped.foot_bodies.(foot).contact_pts(:,...
        biped.foot_bodies.(foot).collision_group{...
            cellfun(@(x) strcmp(x, grp), ...
             biped.foot_bodies.(foot).collision_group_name)});
      offset.(foot).(grp) = mean(gc(1:3,:), 2);
    end
    offset.(foot).center = mean([offset.(foot).toe, offset.(foot).heel], 2);
  end
end
