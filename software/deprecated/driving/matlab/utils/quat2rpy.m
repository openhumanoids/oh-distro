function rpy = quat2rpy(q)

roll_a = 2 * (q(1)*q(2) + q(3)*q(4));
roll_b = 1 - 2 * (q(2)*q(2) + q(3)*q(3));
rpy(1) = atan2(roll_a, roll_b);

pitch_sin = 2 * (q(1)*q(3) - q(4)*q(2));
rpy(2) = asin(pitch_sin);

yaw_a = 2 * (q(1)*q(4) + q(2)*q(3));
yaw_b = 1 - 2 * (q(3)*q(3) + q(4)*q(4));
rpy(3) = atan2(yaw_a, yaw_b);

rpy = rpy(:);
