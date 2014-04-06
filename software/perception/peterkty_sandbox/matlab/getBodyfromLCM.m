function [posek] = getBodyfromLCM()

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();

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
posemsg_dec = bot_core.pose_t(bdmsg.data);
posek = [posemsg_dec.pos(1), posemsg_dec.pos(2), posemsg_dec.pos(3), ...
    posemsg_dec.orientation(1), posemsg_dec.orientation(2), posemsg_dec.orientation(3), posemsg_dec.orientation(4)]';

fprintf('posek: %f %f %f %f %f %f %f\n', posek(1), posek(2), posek(3), posek(4), posek(5), posek(6), posek(7));

end