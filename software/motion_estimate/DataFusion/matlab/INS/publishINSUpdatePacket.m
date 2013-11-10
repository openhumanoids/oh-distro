function publishINSUpdatePacket( msg, posterior, feedbackGain, lc )
%PUBLISHINSUPDATEPACKET Summary of this function goes here
%   Detailed explanation goes here

msg.utime = posterior.utime;

dq = e2q(feedbackGain*posterior.x(1:3));

msg.dQ.w = dq(1);
msg.dQ.x = dq(2);
msg.dQ.y = dq(3);
msg.dQ.z = dq(4);

msg.dbiasGyro_b.x = feedbackGain*posterior.x(4);
msg.dbiasGyro_b.y = feedbackGain*posterior.x(5);
msg.dbiasGyro_b.z = feedbackGain*posterior.x(6);

msg.dPos_l.x = feedbackGain*posterior.x(7);
msg.dPos_l.y = feedbackGain*posterior.x(8);
msg.dPos_l.z = feedbackGain*posterior.x(9);

msg.dVel_l.x = feedbackGain*posterior.x(10);
msg.dVel_l.y = feedbackGain*posterior.x(11);
msg.dVel_l.z = feedbackGain*posterior.x(12);

msg.dbiasAcc_b.x = feedbackGain*posterior.x(13);
msg.dbiasAcc_b.y = feedbackGain*posterior.x(14);
msg.dbiasAcc_b.z = feedbackGain*posterior.x(15);

lc.publish('INS_ERR_UPDATE', msg);

end

