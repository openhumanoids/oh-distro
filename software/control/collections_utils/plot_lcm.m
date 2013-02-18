function plot_lcm(x,y,z, id, name, type, reset)
m = vs.obj_collection_t();

% temp hack as cannot append to empty array 
% Ask Russ to fix this issue
msg = vs.obj_t();
msg.id= 0;
msg.x= 0;
msg.y= 0;
msg.z= 0;
msg.yaw= 0;
msg.pitch= 0;
msg.roll= 0;

msg2 = vs.obj_t();
msg2.id= 0;
msg2.x= 0;
msg2.y= 0;
msg2.z= 0;
msg2.yaw= 0;
msg2.pitch= 0;
msg2.roll= 0;
m.objs =[msg , msg2];


for i=1:size(x,1)
  msg2 = vs.obj_t();
  msg2.id= i;
  msg2.x= x(i);
  msg2.y= y(i);
  msg2.z= z(i);
  msg2.yaw= 0;
  msg2.pitch= 0;
  msg2.roll= 0;
  m.objs = [m.objs msg2];
end

m.id =id;
m.type =type;
m.name =name;
m.reset =logical(reset);
m.nobjs = size(x,1);
m.objs = m.objs(3:end)

lc = lcm.lcm.LCM.getSingleton();
lc.publish('OBJ_COLLECTION', m);