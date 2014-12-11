
x= rand(10,3)




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


m = vs.obj_collection_t();
m.id =9;
m.type =4;
m.name ='sdfsd';
m.reset =logical(1);
m.nobjs = 2;
m.objs =[msg , msg2];


lc.publish('OBJ_COLLECTION', m);
keyboard