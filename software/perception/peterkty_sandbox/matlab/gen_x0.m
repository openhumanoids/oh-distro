function x0 = gen_x0(camposek)
  global bound hbound
  x0 = (rand(3,1)*bound-[hbound,hbound,0]');
  %x0 = quat2dcm(camposek(4:7,1)') \ (x0) + camposek(1:3,1);               % sample from p_x0 (returns column vector)
  x0 = quatrotate(quatinv(camposek(4:7,1)'), (x0)')' + camposek(1:3,1) ;               % sample from p_x0 (returns column vector)
end