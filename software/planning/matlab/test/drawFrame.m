function drawFrame(xyz_quat, channel_name, varargin)
%NOTEST
%this must be preceeded by and succeeded by these two lines
% first:
%lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'channel-name');
% then:
%lcmgl.switchBuffers();

if nargin < 3
  scale = 0.1;
else
  scale = varargin{1};
end
if nargin < 4
  color = [1,0,1];
else
  color = varargin{2};
end

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),channel_name);

res = quat2axis( xyz_quat(4:7) );
lcmgl.glTranslated(xyz_quat(1),xyz_quat(2),xyz_quat(3));
lcmgl.glRotated(res(4)*180/pi, res(1),res(2), res(3));
lcmgl.glScalef(0.1, 0.1, 0.1);
lcmgl.glColor3f(color(1), color(2), color(3) );
lcmgl.sphere([0,0,1], scale, 1200, 1200);
lcmgl.glDrawAxes();
lcmgl.glScalef(10, 10, 10);
%lcmgl.sphere(l_hand_pt, 0.01, 20, 20);
lcmgl.glRotated(-res(4)*180/pi, res(1),res(2), res(3));
lcmgl.glTranslated(-xyz_quat(1),-xyz_quat(2),-xyz_quat(3));

lcmgl.switchBuffers();
