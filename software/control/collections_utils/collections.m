function plot_lcm(x,y,z, id, name, type)
m = vs.obj_collection_t();

msg = vs.obj_t();
msg.id= 5;
msg.x= 5;
msg.y= 5;
msg.z= 5;
msg.yaw= 5;
msg.pitch= 5;
msg.roll= 5;

msg2 = vs.obj_t();
msg2.id= 2;
msg2.x= 10;
msg2.y= 5;
msg2.z= 0;
msg2.yaw= 0;
msg2.pitch= 0;
msg2.roll= 0;
m.objs =[msg , msg2];


for i=1:1000
  msg2 = vs.obj_t();
  msg2.id= i+2;
  msg2.x= x(i,1);
  msg2.y= x(i,2);
  msg2.z= x(i,3);
  msg2.yaw= 0;
  msg2.pitch= 0;
  msg2.roll= 0;
  m.objs = [m.objs msg2];
end
keyboard

m.id =9;
m.type =3;
m.name ='sdfsd';
m.reset =logical(1);
m.nobjs = 1002;

lc.publish('OBJ_COLLECTION', m);