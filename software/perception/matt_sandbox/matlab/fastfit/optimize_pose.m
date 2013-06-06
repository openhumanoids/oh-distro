function result = optimize_pose(dist_xform,pts,pose_init,only_2d)

if (~exist('only_2d','var'))
    only_2d = false;
end

dat.dist_xform = double(dist_xform)/sqrt(sum(size(dist_xform).^2));
dat.pts = pts;
dat.only_2d = only_2d;

if (only_2d)
    x0 = [atan2(pose_init.R(1,2),pose_init.R(1,1));pose_init.T(1:2)];
else
    x0 = [rot2rpy(pose_init.R);pose_init.T(:)];
end

opts = optimset('display','iter');
x = lsqnonlin(@error_func, x0, [], [], opts, dat);
if (only_2d)
    R = axisangle2rot([0;0;1],x(1));
    T = [x(2:3);0];
else
    R = rpy2rot(x(1:3));
    T = x(4:6);
end
result.pose.R = R;
result.pose.T = T;

function e = error_func(x,dat)

if (dat.only_2d)
    R = axisangle2rot([0;0;1],x(1));
    T = [x(2:3);0];
else
    R = rpy2rot(x(1:3));
    T = x(4:6);
end
e = dist_xform_error(dat.dist_xform, dat.pts, R,T,10);
figure(22); plot(e)
