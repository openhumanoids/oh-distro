function r = visualizeCollisionMesh(r, EEid, q, pos, quat, EE_point)
  if nargin < 6, EE_point = [0; 0; 0]; end
  for p = 1:size(pos, 2)
    body = RigidBody();
    body.robotnum = 2;
    body.linkname = sprintf('CollisionGeometry%d', p);
    [r, bodyId] = r.addLink(body);
    T = [quat2rotmat(quat(:,p)), pos(:,p); [0 0 0 1]];
    for i = 1:numel(r.getBody(EEid).collision_geometry)
      geom = r.getBody(EEid).collision_geometry{i};
      geom.T(1:3,4) = geom.T(1:3,4) - EE_point;
      geom.T = T * geom.T;
      r = r.addVisualGeometryToBody(bodyId, geom);
    end
  end
  r = r.compile();
  v = r.constructVisualizer();
  v.draw(0, q)
end
% state = Scenes.getFP('val', r);
% v.draw(0, state);
% for i = 1:numel(r.getBody(bodyId).visual_geometry)
%   r.getBody(bodyId).visual_geometry{i}.T(1:3,4) = r.getBody(bodyId).visual_geometry{i}.T(1:3,4) + [0; 1; 0];
% end