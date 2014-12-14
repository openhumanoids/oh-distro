function view3d(arg)

if (nargin == 0)
    view3d_obj = findobj(allchild(gcf),'Tag','CustomViewer3D');
    if (isempty(view3d_obj))
        do_viewer(gcf,'on');
    else
        do_viewer(gcf,'off');
    end
elseif (nargin == 1)
    switch(lower(arg))
        case 'on'
            do_viewer(gcf,'on');
        case 'off'
            do_viewer(gcf,'off');
        otherwise
            error('Invalid argument; please specify ''on'' or ''off''');
    end
end


function do_viewer(fig,action)

view3d_obj = findobj(allchild(fig),'Tag','CustomViewer3D');

if (strcmp(action,'off'))
   if (isempty(view3d_obj))
      return
   end
   vdata = get(view3d_obj,'UserData');
   uirestore(vdata.uistate);
   delete(view3d_obj);
   return
end

if isempty(view3d_obj)
    make_viewer_object(fig);
end
axis('vis3d');


function view3d_button_down_func(varargin)

view3d_obj  = findobj(allchild(gcf),'Tag','CustomViewer3D');
if isempty(view3d_obj)
   return
end

mouseclick = get(gcf,'SelectionType');
vdata = get(view3d_obj,'UserData');
vdata.oldunits = get(gcf,'Units');
set(gcf,'Units','pixels');
vdata.old_pt = get(gcf,'CurrentPoint');
vdata.old_pose.CameraPosition = get(gca, 'CameraPosition');
vdata.old_pose.CameraTarget = get(gca, 'CameraTarget');
vdata.old_pose.CameraUpVector = get(gca, 'CameraUpVector');
vdata.old_pose.CameraViewAngle = get(gca, 'CameraViewAngle');

if (strcmp(mouseclick,'normal'))
    set(gcf,'WindowButtonMotionFcn',@view3d_rotate_func);
elseif (strcmp(mouseclick,'extend'))
    set(gcf,'WindowButtonMotionFcn',@view3d_dolly_func);
elseif (strcmp(mouseclick,'alt'))
    set(gcf,'WindowButtonMotionFcn',@view3d_pan_func);
end
set(view3d_obj,'UserData',vdata);


function view3d_button_up_func(varargin)

view3d_obj = findobj(allchild(gcf),'Tag','CustomViewer3D');
if isempty(view3d_obj)
   return
end
vdata = get(view3d_obj,'UserData');
set(gcf,'WindowButtonMotionFcn','','Units',vdata.oldunits,'pointer','arrow')


function view3d_rotate_func(varargin)

view3d_obj = findobj(allchild(gcf),'Tag','CustomViewer3D');
vdata = get(view3d_obj,'UserData');

pos = get(gcf,'Position');
radius = hypot(pos(3),pos(4))/2;
radius = min(pos(3:4));
ctr = pos(3:4)/2;
old_pt = vdata.old_pt(:);
new_pt = get(gcf,'CurrentPoint');
old_pt = old_pt(:) - ctr(:);
new_pt = new_pt(:) - ctr(:);
% TODO: enable this if using the sphere method
% if (norm(new_pt) >= radius || norm(old_pt) >= radius)
%     return
% end

set(gca, 'CameraPosition', vdata.old_pose.CameraPosition);
set(gca, 'CameraTarget', vdata.old_pose.CameraTarget);
set(gca, 'CameraUpVector', vdata.old_pose.CameraUpVector);

% TODO: enable this if using the sphere method
% old_ray = [old_pt; sqrt(radius^2-old_pt(1)^2-old_pt(2)^2)];
% new_ray = [new_pt; sqrt(radius^2-new_pt(1)^2-new_pt(2)^2)];
% c1 = cross(old_ray, [0;1;0]);
% c2 = cross(new_ray, [0;1;0]);
% c3 = cross(c2,c1);
% dx_angle = sign(c3(2))*acos(dot(c1,c2)/norm(c1)/norm(c2))*180/pi;
% c1 = cross(old_ray, [1;0;0]);
% c2 = cross(new_ray, [1;0;0]);
% c3 = cross(c1,c2);
% dy_angle = sign(c3(1))*acos(dot(c1,c2)/norm(c1)/norm(c2))*180/pi;

dx_angle = (old_pt(1)-new_pt(1))/radius*180;
dy_angle = (old_pt(2)-new_pt(2))/radius*180;
camorbit(dx_angle, dy_angle, 'camera');


function view3d_dolly_func(varargin)

view3d_obj  = findobj(allchild(gcf),'Tag','CustomViewer3D');
vdata = get(view3d_obj,'UserData');
old_pt = vdata.old_pt;
new_pt = get(gcf,'CurrentPoint');
pos = get(gcf,'Position');
amt = old_pt(2) - new_pt(2);
zoom_factor = 2^(4*amt/pos(4));
if (zoom_factor <= 1e-2)
    return
end

set(gca, 'CameraViewAngle', vdata.old_pose.CameraViewAngle);
camzoom(zoom_factor);


function view3d_pan_func(varargin)

view3d_obj  = findobj(allchild(gcf),'Tag','CustomViewer3D');
vdata = get(view3d_obj,'UserData');
old_pt = vdata.old_pt;
new_pt = get(gcf,'CurrentPoint');
dx = new_pt(1) - old_pt(1);
dy = new_pt(2) - old_pt(2);
set(gca, 'CameraPosition', vdata.old_pose.CameraPosition);
set(gca, 'CameraTarget', vdata.old_pose.CameraTarget);
set(gca, 'CameraUpVector', vdata.old_pose.CameraUpVector);
camdolly(-dx,-dy,0, 'movetarget','pixels');


function view3d_obj = make_viewer_object(fig)

% save the previous state of the figure window
vdata.uistate  = uisuspend(fig);

% the data structure
vdata.old_pt   = [];
vdata.oldunits = [];

% viewer object
view3d_obj = uicontrol('style','text','parent',fig,'Units','Pixels',... 
                      'Visible','off', 'HandleVisibility','off',...
                      'Tag','CustomViewer3D');

% functions
set(fig,'WindowButtonDownFcn',@view3d_button_down_func);
set(fig,'WindowButtonUpFcn',@view3d_button_up_func);
set(fig,'WindowButtonMotionFcn','');
set(fig,'ButtonDownFcn','');
set(fig,'KeyPressFcn','');

set(view3d_obj,'UserData',vdata);
