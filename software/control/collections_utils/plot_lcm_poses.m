function plot_lcm(values , ypr, id, name, type, reset, draw_links, link_id)
% Supported Pose Type Values
% POSE=1, 
% TREE=2, 
% SQUARE=3, 
% POSE3D=4, 
% AXIS3D=5, 
% TAG=6, 
% CAMERA=7, 
% TRIANGLE=8, 
% HEXAGON=9, 
% SONARCONE=10;

m = vs.obj_collection_t();
m.objs = javaArray('vs.obj_t', size(values, 1));
for i=1:size(values,1)
  msg2 = vs.obj_t();
  msg2.id= i-1;
  msg2.x= values(i,1);
  msg2.y= values(i,2);
  msg2.z= values(i,3);
  msg2.yaw= ypr(i,1);
  msg2.pitch= ypr(i,2);
  msg2.roll= ypr(i,3);
  m.objs(i) =msg2;
end

m.id =id;
m.type =type;
m.name =name;
m.reset =logical(reset);
m.nobjs = size(values,1);
%m.objs = m.objs(1:end);

lc = lcm.lcm.LCM.getSingleton();
lc.publish('OBJ_COLLECTION', m);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% LINKS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (draw_links)
link_col = vs.link_collection_t();
link_col.id = link_id;
link_col.name = [name ' (L)'] ;
link_col.reset =logical(reset);

link_col.links =javaArray('vs.link_t', size(values-1, 1));
for i=2:size(values,1)
  link = vs.link_t();
  link.id = i-1;
  link.collection1 = id;
  link.id1 = i-2;
  link.collection2 = id;
  link.id2 = i-1;
  link_col.links(i-1)=link;
end 
link_col.nlinks=size(values,1)-1;

lc.publish('LINK_COLLECTION',link_col)
end