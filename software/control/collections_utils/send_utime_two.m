function send_utime_two(channel , utime_sim, utime_wall)
m = drc.utime_two_t();
m.utime_wall =utime_wall;
m.utime_sim =int64(utime_sim) ;
lc = lcm.lcm.LCM.getSingleton();
lc.publish(channel, m); 

%          send_utime_two('LATENCY_CONTROL_B', (tt+t_offset)*1E6, bot_timestamp_now);                
%          u = obj.controller.output(tt,[],vertcat(input_frame_data{:}));
%          send_utime_two('LATENCY_CONTROL_C', (tt+t_offset)*1E6, bot_timestamp_now);                
%          obj.controller_output_frame.publish(tt+t_offset,u,defaultChannel(obj.controller_output_frame));
%          send_utime_two('LATENCY_CONTROL_D', (tt+t_offset)*1E6, bot_timestamp_now);   