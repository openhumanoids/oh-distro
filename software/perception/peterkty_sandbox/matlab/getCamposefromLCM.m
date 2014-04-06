function [camposek] = getCamposefromLCM()

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();

lc.subscribe('BODY_TO_CAMERARHAND', aggregator);

while true
    disp waiting_BODY_TO_CAMERARHAND
    millis_to_wait = 1000;
    cammsg = aggregator.getNextMessage(millis_to_wait);
    if ~isempty(cammsg)
        break
    end
end
lc.unsubscribe('BODY_TO_CAMERARHAND', aggregator);

lc.subscribe('POSE_BODY', aggregator);

while true
    disp waiting_POSE_BODY
    millis_to_wait = 1000;
    bdmsg = aggregator.getNextMessage(millis_to_wait);
    if ~isempty(bdmsg)
        break
    end
end
lc.unsubscribe('POSE_BODY', aggregator);


%lc.close();
camposemsg_dec = vicon.body_t(cammsg.data);
camposek = [camposemsg_dec.trans(1), camposemsg_dec.trans(2), camposemsg_dec.trans(3), ...
    camposemsg_dec.quat(1), camposemsg_dec.quat(2), camposemsg_dec.quat(3), camposemsg_dec.quat(4)]';

fprintf('camposek: %f %f %f %f %f %f %f\n', camposek(1), camposek(2), camposek(3), camposek(4), camposek(5), camposek(6), camposek(7));

posemsg_dec = bot_core.pose_t(bdmsg.data);
posek = [posemsg_dec.pos(1), posemsg_dec.pos(2), posemsg_dec.pos(3), ...
    posemsg_dec.orientation(1), posemsg_dec.orientation(2), posemsg_dec.orientation(3), posemsg_dec.orientation(4)]';

fprintf('posek: %f %f %f %f %f %f %f\n', posek(1), posek(2), posek(3), posek(4), posek(5), posek(6), posek(7));

% get transform relative to world
comcamposek = camposek;
comcamposek(4:7,1) = quatmultiply(posek(4:7,1)', camposek(4:7,1)');
comcamposek(1:3,1) = quatrotate(quatinv(posek(4:7,1)'), camposek(1:3,1)')' + posek(1:3,1);
    
camposek = comcamposek;

end