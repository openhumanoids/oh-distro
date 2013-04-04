function send_status(system,importance, frequency, value)
status = drc.system_status_t();
status.utime  = etime(clock,[1970 1 1 0 0 0])*1000000;
status.system = system;
status.importance = importance;
status.frequency  = 0;
status.value =value;
lc = lcm.lcm.LCM.getSingleton();
lc.publish('SYSTEM_STATUS', status);
pause(0.25)