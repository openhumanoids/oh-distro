function testTerrainMap

lc = lcm.lcm.LCM.getSingleton();
robot_state_mon = drake.util.MessageMonitor(drc.robot_state_t,'utime');
lc.subscribe('EST_ROBOT_STATE',robot_state_mon);
robot_state_coder = RobotStateCoder('atlas',r.getStateFrame.coordinates(1:getNumDOF(r)));
map = DRCTerrainMap;

% first wait for at least one state message
d = getNextMessage(robot_state_mon);
x0 = robot_state_coder.decode(d);

[X,Y] = meshgrid(x0(1)+[-10:.2:10],x0(2)+[-10:.2:10]);
[z,n] = getHeight(map,[X(:),Y(:)]');
Z = reshape(z,size(X));
U = reshape(n(1,:),size(X));
V = reshape(n(2,:),size(X));
W = reshape(n(3,:),size(X));

figure(1);
mesh(X,Y,Z);
quiver3(X,Y,Z,U,V,W);


end