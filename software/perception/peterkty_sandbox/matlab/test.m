figure(2);
title('object pose estimate');
e = 1000;
plot3(xh(1,2:e), xh(2,2:e), xh(3,2:e));
for i=2:e
  text(xh(1,i), xh(2,i), xh(3,i), sprintf('%d', i));
end
xlabel('x'); ylabel('y'); zlabel('z');
axis equal

figure(3);

ccampose = campose;
%ccampose(4:7, :) = [ccampose(7,:); ccampose(4:6,:)];
for i=3:e
  %ccampose(1:3,i) = quatrotate(quatinv(campose(4:7,i)'), ([0,0,0]'-campose(1:3, i))')';
  ccampose(1:3,i) = quatrotate(quatinv(ccampose(4:7,i)'), ([0,0,0]')')'+campose(1:3, i);
  %ccampose(1:3,i) = quatrotate(quatinv(campose(4:7,i)'), ([0,0,0]')')'+campose(1:3, i);
end

title('camera pose');
plot3(ccampose(1,3:e), ccampose(2,3:e), ccampose(3,3:e));
for i=3:e
  text(ccampose(1,i), ccampose(2,i), ccampose(3,i), sprintf('%d', i));
end
xlabel('x'); ylabel('y'); zlabel('z');
axis equal