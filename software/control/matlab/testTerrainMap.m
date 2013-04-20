function testTerrainMap

lc = lcm.lcm.LCM.getSingleton();
robot_state_mon = drake.util.MessageMonitor(drc.robot_state_t,'utime');
lc.subscribe('EST_ROBOT_STATE',robot_state_mon);
robot_state_coder = RobotStateCoder('atlas',{''});
map = DRCTerrainMap;
map_mon = drake.util.MessageMonitor(drc.map_image_t,'utime');
lc.subscribe('MAP_DEPTH',map_mon);

fprintf('waiting for a robot state message...');
d=[];
while(isempty(d))
  d = getNextMessage(robot_state_mon,100);
  drawnow;
end
x0 = robot_state_coder.decode(d).val;
fprintf(1,'received!\n');

while (1) 
  d = getNextMessage(map_mon,100);  % wait for gui request to start
  if isempty(d), drawnow; continue; end
  
  [X,Y] = meshgrid(x0(1)+[-10:.2:10],x0(2)+[-10:.2:10]);
  [z,n] = getHeight(map,[X(:),Y(:)]');
  
  Z = reshape(z,size(X));
  U = reshape(n(1,:),size(X));
  V = reshape(n(2,:),size(X));
  W = reshape(n(3,:),size(X));
  
  figure(1); clf; hold on;
  mesh(X,Y,Z);
  quiver3(X,Y,Z,U,V,W);
end

end
