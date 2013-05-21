function collisionAvoidanceServer(colAvCon,channel_out)
  % listens for drc_action_sequence_t messages and, upon receipt, adds collision
  % avoidance contact goals and republishes the drc_action_sequence_t

  lc = lcm.lcm.LCM.getSingleton(); %('udpm://239.255.76.67:7667?ttl=1');
  channel_in = 'REQUEST_MOTION_PLAN_FOR_ACTION_SEQUENCE';
  if nargin < 2
    channel_out = 'DRAKE_REQUEST_MOTION_PLAN_FOR_ACTION_SEQUENCE';
  end

  % construct lcm input monitor
  monitor = drake.util.MessageMonitor(drc.action_sequence_t(),'utime');
  lc.subscribe(channel_in,monitor);

  timeout=10;
  display('Listening ...');
  while (1)
    warning on
    data = getNextMessage(monitor,timeout);
    if ~isempty(data)
      display('Received action sequence message ...')
      msg = drc.action_sequence_t(data);
      pub = ActionSequencePublisher(channel_out,colAvCon,[],msg);
      display('Publishing action sequence message ...')
      pub.publish(0);
      display('Listening ...');
    end
  end
end
