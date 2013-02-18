function plot_lcm_points(values,colors_in, id, name, type, reset)
% Supported Type Values
% POINT=1,
% LINE_STRIP=2,
% LINE_LOOP=3,
% LINES=4,
% TRIANGLE_STRIP=5,
% TRIANGLE_FAN=6,
% TRIANGLES=7,
% QUAD_STRIP=8,
% QUADS=9,
% POLYGON=10;


% temp hack as cannot append to empty array 
% Ask Russ to fix this issue
msg = vs.obj_t();
msg.id= 1;
msg.x= 0;
msg.y= 0;
msg.z= 0;
msg.yaw= 0;
msg.pitch= 0;
msg.roll= 0;

msg2 = vs.obj_t();
msg2.id= 1;
msg2.x= 0;
msg2.y= 0;
msg2.z= 0;
msg2.yaw= 0;
msg2.pitch= 0;
msg2.roll= 0;

m = vs.obj_collection_t();
m.objs =[msg , msg2];
m.id =100000;
m.type =5; % rgb triad
m.name ='Null Pose (Matlab)';
m.reset =logical(reset);
m.nobjs = 1; % TODO fix this properly

lc = lcm.lcm.LCM.getSingleton();
lc.publish('OBJ_COLLECTION', m);



c = vs.point3d_list_collection_t();
c.id = id+100000;
c.name = name;
c.type = type;
c.reset = logical(1);


l = vs.point3d_list_t();
l.id =0;
l.collection = 100000;
l.element_id = 1;
l.ncolors =0;
l.nnormals=0;
l.npointids=0;


c.nlists = 1;
c.point_lists = [l l]

pt1 = vs.point3d_t();
pt1.x =1;
pt1.y =2;
pt1.z =3;
pt2 = vs.point3d_t();
pt2.x =4;
pt2.y =2;
pt2.z =3;
l.points = [pt1 , pt2]

col1 = vs.color_t();
col1.r =1;
col1.g =2;
col1.b =3;
col2 = vs.color_t();
col2.r =4;
col2.g =2;
col2.b =3;
l.colors = [col1 , col2];


for i=1:size(values,1)
  pt = vs.point3d_t();
  pt.x= values(i,1);
  pt.y= values(i,2);
  pt.z= values(i,3);
  l.points= [l.points pt];

  col = vs.color_t();
  col.r= colors_in(i,1);
  col.g= colors_in(i,2);
  col.b= colors_in(i,3);
  l.colors= [l.colors col];
end
l.npoints = size(values,1);
l.points = l.points(3:end);

l.ncolors = size(colors_in,1);
l.colors = l.colors(3:end);

lc.publish('POINTS_COLLECTION', c);
