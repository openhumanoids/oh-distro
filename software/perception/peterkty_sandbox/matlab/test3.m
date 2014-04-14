% addpath_drake
% addpath_control
% javaaddpath('/home/drc/drc/software/build/share/java/lcmtypes_trackers.jar');
% javaaddpath('/home/drc/drc/software/build/share/java/lcmtypes_vicon.jar');
% javaaddpath('/home/drc/drc/software/build/share/java/lcmtypes_bot2-core.jar');
% 
% javaaddpath('/usr/local/share/java/lcm.jar');

while 1
    %camposek = getCamposefromLCM();
    palmposek = getPalmposefromLCM();
    
    
    x_palm = [0 0 0]';
    x_palm = quatrotate(quatinv(palmposek(4:7,1)'), x_palm')' + palmposek(1:3);
    
    lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'body');
    lcmgl.glColor3f(1,0,1);
    lcmgl.sphere(x_palm,0.03,100,100);
    lcmgl.switchBuffers;
    %break;
end
