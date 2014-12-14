function publishINSUpdatePacket( msg, posterior, feedbackGain, Measurement, lc )
%PUBLISHINSUPDATEPACKET Summary of this function goes here
%   Detailed explanation goes here

msg.utime = posterior.utime;

msg.dE_l.x = - feedbackGain*posterior.x(1);
msg.dE_l.y = - feedbackGain*posterior.x(2);
msg.dE_l.z = - feedbackGain*posterior.x(3);

msg.dbiasGyro_b.x = feedbackGain*posterior.x(4);
msg.dbiasGyro_b.y = feedbackGain*posterior.x(5);
msg.dbiasGyro_b.z = feedbackGain*posterior.x(6);

msg.dVel_l.x = - feedbackGain*posterior.x(7);
msg.dVel_l.y = - feedbackGain*posterior.x(8);
msg.dVel_l.z = - feedbackGain*posterior.x(9);

msg.dbiasAcc_b.x = feedbackGain*posterior.x(10);
msg.dbiasAcc_b.y = feedbackGain*posterior.x(11);
msg.dbiasAcc_b.z = feedbackGain*posterior.x(12);

msg.dPos_l.x = - feedbackGain*posterior.x(13);
msg.dPos_l.y = - feedbackGain*posterior.x(14);
msg.dPos_l.z = - feedbackGain*posterior.x(15);

msg.V_resid_l.x = Measurement.velocityResidual(1);
msg.V_resid_l.y = Measurement.velocityResidual(2);
msg.V_resid_l.z = Measurement.velocityResidual(3);

msg.P_resid_l.x = 0;
msg.P_resid_l.y = 0;
msg.P_resid_l.z = 0;

lc.publish('INS_ERR_UPDATE', msg);

end

