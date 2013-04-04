% % Use a base null pose
% msg = vs.obj_t();
% msg.id= 1;
% msg.x= 0;
% msg.y= 0;
% msg.z= 0;
% msg.yaw= 0;
% msg.pitch= 0;
% msg.roll= 0;
% m = vs.obj_collection_t();
% m.objs = javaArray('vs.obj_t', size(1, 1));
% m.objs(1) = msg;
% m.id =100000;
% m.type =5; % rgb triad
% m.name ='Null Pose (Matlab)';
% m.reset =logical(1);
% m.nobjs = 1; % TODO fix this properly
% lc = lcm.lcm.LCM.getSingleton();
% lc.publish('OBJ_COLLECTION', m);
% 
% 
% 
disp('dfdf')
for i=1:100
  i
status = drc.system_status_t();
status.utime  = etime(clock,[1970 1 1 0 0 0])*1000000;
status.system = mod(i,5);
mod(i,5)
status.importance = mod(i,3);
status.frequency  = 0;
status.value = ['Message here ' num2str(i) ' ' num2str(mod(i,5)) ...
  ' ' num2str(mod(i,3))];
lc = lcm.lcm.LCM.getSingleton();
lc.publish('SYSTEM_STATUS', status);
pause(0.25)
end
