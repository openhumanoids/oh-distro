function plot_lcm_points(values,colors_in, id, name, type, reset)
% Supported Point Type Values
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

% fail quietly if the dependency is not met.
% note: i added this because the visualization pod has gross dependencies,
% crawling into isam, etc.  that's a lot if you just want to print some
% points. - Russ
if ~exist('vs','class'), return; end  

% Use a base null pose
msg = vs.obj_t();
msg.id= 1;
msg.x= 0;
msg.y= 0;
msg.z= 0;
msg.yaw= 0;
msg.pitch= 0;
msg.roll= 0;
m = vs.obj_collection_t();
m.objs = javaArray('vs.obj_t', size(1, 1));
m.objs(1) = msg;
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
c.reset = logical(reset);

l = vs.point3d_list_t();
l.id =now() * 24 * 60 * 60;
l.collection = 100000;
l.element_id = 1;
l.ncolors =0;
l.nnormals=0;
l.npointids=0;
c.point_lists = javaArray('vs.point3d_list_t', size(1, 1));
c.nlists = 1;
c.point_lists(1) = l;


l.points = javaArray('vs.point3d_t', size(values, 1));
l.colors = javaArray('vs.color_t', size(values, 1));
for i=1:size(values,1)
  pt = vs.point3d_t();
  pt.x= values(i,1);
  pt.y= values(i,2);
  pt.z= values(i,3);
  l.points(i)= pt;

  col = vs.color_t();
  col.r= colors_in(i,1);
  col.g= colors_in(i,2);
  col.b= colors_in(i,3);
  l.colors(i)= col;
end
l.npoints = size(values,1);
l.ncolors = size(colors_in,1);
lc.publish('POINTS_COLLECTION', c);