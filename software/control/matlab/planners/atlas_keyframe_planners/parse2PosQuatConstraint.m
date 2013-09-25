function constraint_cell = parse2PosQuatConstraint(robot,body,pts,pose,pos_tol,quat_tol,tspan)
  % construct a WorldPositionConstraint and a WorldQuatConstraint
  % given pose
  % @param robot          robot
  % @param body           body index
  % @param pts            body points
  % @param pose           A 7 x npts matrix
  % @pos_tol              A scalar or a 3 x npts matrix, the position tolerance
  % @quat_tol             A scalar, the tolerance for WorldQuatConstraint
  % @tspan                Optional time span
  if (nargin == 6)
    tspan = [-inf,inf];
  end
  npts = size(pts,2);
  if(size(pts,1) ~= 3)
    error('Incorrect points, must have 3 rows');
  end
  sizecheck(pose,[7,npts]);
  pos_min = pose(1:3)-pos_tol;
  pos_max = pose(1:3)+pos_tol;
  quat_des = pose(4:7,1);
  constraint_cell = {WorldPositionConstraint(robot,body,pts,pos_min,pos_max,tspan),...
    WorldQuatConstraint(robot,body,quat_des,quat_tol,tspan)};
end

