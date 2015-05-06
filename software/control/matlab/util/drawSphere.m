function drawSphere(pos)
  if length(pos) == 2
    pos = [pos;0];
  end
  lcmgl = LCMGLClient;
  lcmgl.glColor3f(1,0,0);
  lcmgl.sphere(pos,0.02,20,20);
  lcmgl.switchBuffers;
end