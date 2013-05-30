function pos = fitPoseToNormal(pos, normal)

sizecheck(pos, [6,1]);

M0 = rpy2rotmat(pos(4:6));
x0 = M0 * [1;0;0];
z0 = M0 * [0;0;1];
ax = fastCross3(z0, normal);
costheta = dot(normal, z0) / norm(normal);
theta = real(acos(costheta));
q = axis2quat([ax;theta]);
Mf = quat2rotmat(q);
xf = Mf * x0;
zf = normal;
yf = fastCross3(zf, xf);
Mx = xf / norm(xf);
Mz = fastCross3(xf, yf);
Mz = Mz / norm(Mz);
My = fastCross3(Mz, xf);
My = My / norm(My);
M = [Mx, My, Mz];
new_rpy = rotmat2rpy(M);
if ~any(isnan(new_rpy))
  pos(4:5) = new_rpy(1:2);
end
