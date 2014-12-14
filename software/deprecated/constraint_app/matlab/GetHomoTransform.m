function T = GetHomoTransform(xyzrpy)

    if length(xyzrpy)==6
        %row pose is [pos, rpy]
        T = eye(4);
        T(1:3,1:3) = bot_quat_to_matrix(bot_roll_pitch_yaw_to_quat(xyzrpy(4:6)));
        T(1:3,4) = xyzrpy(1:3)';  
    else
        error('wrong size for input, should be [x,y,z,r,p,y]')
    end