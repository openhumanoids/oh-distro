function result = optimize_pose_with_directions(dist_inds,pts,pose_init)

max_iter = 100;

R = pose_init.R;
T = pose_init.T(:);

err_prev = 1e10;
for iter = 1:max_iter
    [p0,p1] = point_pairs_from_dist_inds(dist_inds,pts,R,T);
    d = p1-p0;
    figure(11); myplot3(p1,'r.'); hold on; myplot3(p0,'b.'); hold off; axis equal; drawnow; pause(0);
    [R_new,T_new] = align_pts_3d(p0,p1);
    P = [R_new,T_new(:);0,0,0,1]*[R,T(:);0,0,0,1];
    R = P(1:3,1:3);
    T = P(1:3,4);
    fprintf('err %f pts %d %f %f %f\n', sqrt(sum(d(:).^2)), size(p0,1), T);
    err_cur = sum(d(:).^2);
    if (abs(err_cur-err_prev) < 1e-6)
        break;
    end
    err_prev = err_cur;
end

result.pose.R = R;
result.pose.T = T;
result.pose.errors = d;
result.pose.iters = iter;
