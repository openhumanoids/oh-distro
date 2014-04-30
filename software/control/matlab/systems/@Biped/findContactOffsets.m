function offset = findContactOffsets(biped)

	typecheck(biped,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});

  foot_bodies = struct('right', biped.manip.body(biped.foot_bodies_idx.right),...
                       'left', biped.manip.body(biped.foot_bodies_idx.left));
  for f = {'right', 'left'}
    foot = f{1};
	  for g = {'toe', 'heel'}
      grp = g{1};
      gc = foot_bodies.(foot).getTerrainContactPoints(grp);
      if isempty(gc)
        error('There is no collision group by the name: %s', grp);
      end
      offset.(foot).(grp) = mean(gc(1:3,:), 2);
    end
    offset.(foot).center = mean([offset.(foot).toe, offset.(foot).heel], 2);
  end
end
