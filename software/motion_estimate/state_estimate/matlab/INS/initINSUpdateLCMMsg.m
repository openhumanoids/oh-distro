function [ msg ] = initINSUpdateLCMMsg( )
%PREPAREINSUPDATELCMMSG Summary of this function goes here
%   Detailed explanation goes here

msg = drc.ins_update_packet_t();

msg.utime = 0;

msg.dE_l = drc.vector_3d_t();
msg.dbiasGyro_b = drc.vector_3d_t();
msg.dPos_l = drc.vector_3d_t();
msg.dVel_l = drc.vector_3d_t();
msg.dbiasAcc_b = drc.vector_3d_t();

msg.V_resid_l = drc.vector_3d_t();
msg.P_resid_l = drc.vector_3d_t();

end

