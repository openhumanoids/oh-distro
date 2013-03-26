function Xo = footContact2Orig(biped, Xc, group_name, is_right_foot)

	Xo = zeros(size(Xc));
	if is_right_foot
		foot_name = 'right';
	else
		foot_name = 'left';
	end

	offs = biped.foot_contact_offsets.(foot_name).(group_name);

	for j = 1:length(Xc(1,:))
		M = makehgtform('xrotate', Xc(4, j), 'yrotate', Xc(5, j), 'zrotate', Xc(6, j));
		d = M * [offs; 1];
		Xo(:, j) = [Xc(1:3, j) - d(1:3); Xc(4:end, j)];
	end
end