function utime = get_timestamp_now()
 % equivalent to bot_timestamp_now()
 % returns utime in usec
 
    [stat,res]=system('date +%s');   % get sec since the epoch 1 JAN 1970 00:00 UTC
    [stat,res2]=system('date +%N'); % get nano sec
    val=str2num(res);
    val2=str2num(res2);
    utime = [val+1e-9*val2]*1e6;

end
