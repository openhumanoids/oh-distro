% addpath_drake
% addpath_control
% javaaddpath('/home/drc/drc/software/build/share/java/lcmtypes_trackers.jar');
% javaaddpath('/home/drc/drc/software/build/share/java/lcmtypes_vicon.jar');
% javaaddpath('/home/drc/drc/software/build/share/java/lcmtypes_bot2-core.jar');
% 
% javaaddpath('/usr/local/share/java/lcm.jar');

while 1
    posek = getBodyfromLCM();
    camposek = getCamposefromLCM();
    
    x_cam = [0 0.3 0]';
    comcamposek = camposek;
    comcamposek(4:7,1) = quatmultiply(posek(4:7,1)', camposek(4:7,1)');
    comcamposek(1:3,1) = quatrotate(quatinv(posek(4:7,1)'), camposek(1:3,1)')' + posek(1:3,1);
    
    %x_body = quatrotate(quatinv(camposek(4:7,1)'), (x_cam+camposek(1:3))')';
    x_body = quatrotate(quatinv(camposek(4:7,1)'), (x_cam)')' +camposek(1:3);
    %x_body = quatrotate(camposek(4:7,1)', x_cam')' + camposek(1:3);
    x_world = quatrotate(quatinv(posek(4:7,1)'), x_body')' + posek(1:3,1); 
    
    x_world = quatrotate(quatinv(comcamposek(4:7,1)'), x_cam')' + comcamposek(1:3,1); 
    
    lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'body');
    lcmgl.glColor3f(1,0,1);
    lcmgl.sphere(posek(1:3, 1),0.2,100,100);
    lcmgl.sphere(x_world(1:3, 1),0.1,100,100);
    lcmgl.switchBuffers;
    break;
end