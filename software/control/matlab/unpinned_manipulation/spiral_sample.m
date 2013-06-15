function pos = spiral_sample(spiral_center,spiral_axis,n_sample,options)
if(~isfield(options,'type'))
  options.type = 'Archimedean';
end

n_rotation = 2;
if(strcmp(options.type,'Archimedean'))
  theta = linspace(0,n_rotation*2*pi,n_sample);
  rotation_vel = 0.002;
  r = rotation_vel*theta;
end

spiral1 = bsxfun(@times,spiral_center,ones(1,n_sample))+bsxfun(@times,r,ones(3,1)).*[cos(theta);zeros(1,n_sample);sin(theta)];
rotation_axis = cross([0;1;0],spiral_axis);
if(norm(rotation_axis) == 0)
  pos = spiral1;
else
  rotation_axis = rotation_axis/norm(rotation_axis);
  rotation_angle = acos([0;1;0]'*spiral_axis);
  rotation_mat = axis2rotmat([rotation_axis;rotation_angle]);
  pos = rotation_mat*spiral1;
end
end