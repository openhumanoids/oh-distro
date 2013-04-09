function testContactOffset()

	options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
  r = removeCollisionGroupsExcept(r,{'heel','toe'});
  r = compile(r);

  for i = 1:10
		for grp = {'heel', 'toe', 'center'}
			g = grp{1};
			for is_right_foot = [0, 1]
				lb = [-1; -1; -1; -pi; -pi; -pi];
				ub = [1;1;1;pi;pi;pi];
				X = rand(6, 1) .* (ub - lb) + lb;
				assert(all(X - r.footContact2Orig(r.footOrig2Contact(X, g, is_right_foot), g, is_right_foot) < 1e-6));
			end
		end
	end
disp('pass')
end
