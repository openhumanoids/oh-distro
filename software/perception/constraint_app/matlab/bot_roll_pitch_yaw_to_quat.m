function q = bot_roll_pitch_yaw_to_quat (rpy)

    roll = rpy(1);
    pitch = rpy(2);
    yaw = rpy(3);

    halfroll = roll / 2;
    halfpitch = pitch / 2;
    halfyaw = yaw / 2;

    sin_r2 = sin (halfroll);
    sin_p2 = sin (halfpitch);
    sin_y2 = sin (halfyaw);

    cos_r2 = cos (halfroll);
    cos_p2 = cos (halfpitch);
    cos_y2 = cos (halfyaw);

    q(1) = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2;
    q(2) = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2;
    q(3) = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2;
    q(4) = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2;
