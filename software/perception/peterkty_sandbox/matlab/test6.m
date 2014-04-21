campose = zeros(3,0);
i = 0;
    i = i+1;
    
    %campose(1:3, i) = camposek(1:3,1);
    while 1
    pose = bot_frames_transform_vec_mex('LHAND_FACE','local',[0 0 0]);
    lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'body');
    lcmgl.glColor3f(0,1,0);
    lcmgl.sphere(pose,0.03,100,100);
    lcmgl.switchBuffers;
    pause(0.1)
    end
