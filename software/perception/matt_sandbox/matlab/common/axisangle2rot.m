function R = axisangle2rot(ax,ang)

ax = ax/(norm(ax)+1e-10);
R = quat2rot([cos(ang/2),sin(ang/2)*ax(:)']);
