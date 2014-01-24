function [updatePacket] = receiveINSUpdatePacket( aggregator )
%PUBLISHINSUPDATEPACKET Summary of this function goes here
%   Detailed explanation goes here


% lc.publish('INS_ERR_UPDATE', msg);
while true
    %disp waiting
    millis_to_wait = 1;
    msg = aggregator.getNextMessage(millis_to_wait);
    if length(msg) > 0
        break
    end
end

lcmmsg = drc.ins_update_packet_t(msg.data);

updatePacket.utime = lcmmsg.utime;

updatePacket.dE_l = [lcmmsg.dE_l.x;...
                     lcmmsg.dE_l.y;...
                     lcmmsg.dE_l.z];

updatePacket.dbiasGyro_b = [lcmmsg.dbiasGyro_b.x;...
                            lcmmsg.dbiasGyro_b.y;...
                            lcmmsg.dbiasGyro_b.z];
                 
updatePacket.dVel_l = [lcmmsg.dVel_l.x;...
                       lcmmsg.dVel_l.y;...
                       lcmmsg.dVel_l.z];

updatePacket.dbiasAcc_b = [lcmmsg.dbiasAcc_b.x;...
                           lcmmsg.dbiasAcc_b.y;...
                           lcmmsg.dbiasAcc_b.z];

updatePacket.dPos_l = [lcmmsg.dPos_l.x;...
                       lcmmsg.dPos_l.y;...
                       lcmmsg.dPos_l.z];



end

