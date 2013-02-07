function [pos, width] = getAtlasFeetPos(r, q0)

typecheck(r,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
typecheck(q0,'numeric');
sizecheck(q0,[r.getNumDOF,1]);

kinsol = doKinematics(r,q0);
rfoot_body = findLink(r,'r_foot');
lfoot_body = findLink(r,'l_foot');

rfoot0 = forwardKin(r,kinsol,rfoot_body,[0;0;0],true);
lfoot0 = forwardKin(r,kinsol,lfoot_body,[0;0;0],true);

gc = r.contactPositions(q0);

% compute desired COM projection
% assumes minimal contact model for now
k = convhull(gc(1:2,1:4)');
lfootcen0 = [mean(gc(1:2,k),2);0];
k = convhull(gc(1:2,5:8)');
rfootcen0 = [mean(gc(1:2,4+k),2);0];
roffset = rfootcen0 - rfoot0(1:3);
loffset = lfootcen0 - lfoot0(1:3);

function pos = rfootCenter(rfootpos)
  yaw = rfootpos(6);
  offset = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)] * roffset(1:2);
  pos = rfootpos(1:2)+offset;
end    
  
function pos = lfootCenter(lfootpos)
  yaw = lfootpos(6);
  offset = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)] * loffset(1:2);
  pos = lfootpos(1:2)+offset;
end

function pos = feetCenter(rfootpos,lfootpos)
  rcen = rfootCenter(rfootpos);
  lcen = lfootCenter(lfootpos);
  pos = mean([rcen,lcen],2);
end

rfootpos = [rfoot0, rfoot0];
lfootpos = [lfoot0, lfoot0];


p0 = feetCenter(rfoot0, lfoot0);
pos = [p0; 0; 0; 0; atan2(lfoot0(2) - rfoot0(2), lfoot0(1) - rfoot0(1)) - pi/2];
width = sqrt(sum((rfoot0 - lfoot0) .^ 2));

end