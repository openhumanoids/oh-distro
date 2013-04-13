function send_controller_status(status_str)
msg = drc.controller_status_t();
msg.utime  = etime(clock,[1970 1 1 0 0 0])*1000000;
msg.state =0;
if ( strcmp(status_str,'standing') )
  msg.state = 1;
elseif ( strcmp(status_str,'walking') )
  msg.state = 2;
elseif ( strcmp(status_str,'harnessed') )
  msg.state = 3;
end

lc = lcm.lcm.LCM.getSingleton();
lc.publish('CONTROLLER_STATUS', msg);

