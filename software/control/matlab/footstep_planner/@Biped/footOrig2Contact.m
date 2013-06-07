function Xc = footOrig2Contact(biped, Xo, group_name, is_right_foot)

if strcmp(group_name, 'orig')
	Xc = Xo;
else
	Xc = zeros(size(Xo));
	if is_right_foot
		foot_name = 'right';
	else
		foot_name = 'left';
	end

	offs = biped.foot_contact_offsets.(foot_name).(group_name);

	for j = 1:length(Xo(1,:))
    M = rpy2rotmat(Xo(4:6,j));
    d = M * offs;
% 		M = makehgtform('xrotate', Xo(4, j), 'yrotate', Xo(5, j), 'zrotate', Xo(6, j));
% 		d = M * [offs; 1];
		Xc(:, j) = [Xo(1:3, j) + d(1:3); Xo(4:end, j)];
	end
end
end