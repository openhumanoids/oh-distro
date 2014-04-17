campose = zeros(3,0);
i = 0;
while 1
    i = i+1;
    camposek = getCamposefromLCM();
    
    campose(1:3, i) = camposek(1:3,1);
    
%     lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'body');
%     lcmgl.glColor3f(1,0,1);
%     lcmgl.sphere(x_palm,0.03,100,100);
%     lcmgl.switchBuffers;
    figure(5); hold on;
    scatter3(camposek(1), camposek(2), camposek(3));
end
