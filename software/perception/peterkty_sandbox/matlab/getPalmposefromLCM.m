function [palmposek] = getPalmposefromLCM()

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();

lc.subscribe('BODY_TO_RPALM', aggregator);
while true
    disp waiting_BODY_TO_RPALM
    millis_to_wait = 1000;
    palmmsg = aggregator.getNextMessage(millis_to_wait);
    if ~isempty(palmmsg)
        break
    end
end
lc.unsubscribe('BODY_TO_RPALM', aggregator);

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
palmposemsg_dec = vicon.body_t(palmmsg.data);
palmposek = [palmposemsg_dec.trans(1), palmposemsg_dec.trans(2), palmposemsg_dec.trans(3), ...
    palmposemsg_dec.quat(1), palmposemsg_dec.quat(2), palmposemsg_dec.quat(3), palmposemsg_dec.quat(4)]';

fprintf('palmposek: %f %f %f %f %f %f %f\n', palmposek(1), palmposek(2), palmposek(3), palmposek(4), palmposek(5), palmposek(6), palmposek(7));

posemsg_dec = bot_core.pose_t(bdmsg.data);
posek = [posemsg_dec.pos(1), posemsg_dec.pos(2), posemsg_dec.pos(3), ...
    posemsg_dec.orientation(1), posemsg_dec.orientation(2), posemsg_dec.orientation(3), posemsg_dec.orientation(4)]';

fprintf('posek: %f %f %f %f %f %f %f\n', posek(1), posek(2), posek(3), posek(4), posek(5), posek(6), posek(7));

% get transform relative to world
compalmposek = palmposek;
compalmposek(4:7,1) = quatmultiply(posek(4:7,1)', palmposek(4:7,1)');
compalmposek(1:3,1) = quatrotate(quatinv(posek(4:7,1)'), palmposek(1:3,1)')' + posek(1:3,1);
    
palmposek = compalmposek;

end