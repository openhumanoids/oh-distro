function result = align_coarse_to_fine(pts_model,pts_data,res_range,pose_init)

if (~exist('pose_init','var'))
    res = res_range(1);
    [dist_xform,world_to_vol,dist_inds] = create_dist_xform(pts_model,res,10);
    dist_xform = dist_xform/sqrt(sum(size(dist_xform).^2));
    avg_model = mean(pts_model,1);
    pts_data_dec = decimate_points(pts_data,res);
    angle_step = 30;
    roll = -180:angle_step:179;
    pitch = -90:angle_step:89;
    yaw = -180:angle_step:179;
    best_error = 1e10;
    for i = 1:numel(roll)
        for j = 1:numel(pitch)
            for k = 1:numel(yaw)
                rpy = [roll(i);pitch(j);yaw(k)]*pi/180;
                R = rpy2rot(rpy);
                pts = pts_data_dec*R';
                T = avg_model - mean(pts,1);
                pts = pts + repmat(T(:)',[size(pts,1),1]);
                pts = [pts,ones(size(pts,1),1)]*world_to_vol(1:3,:)';
                
                [p0,p1] = point_pairs_from_dist_inds(dist_inds,pts,eye(3),[0;0;0]);
                [R_opt,T_opt] = align_pts_3d(p0,p1);
                
                e = dist_xform_error(dist_xform,pts,R_opt,T_opt,10);
                e = sqrt(sum(e.^2));
                if (e < best_error)
                    fprintf('err %f rpy %f %f %f\n', e, rpy*180/pi);
                    best_error = e;
                    best_P = world_to_vol\[R_opt,T_opt(:);0,0,0,1]*world_to_vol*[R,T(:);0,0,0,1];
                end
            end
        end
    end
    pose_init.R = best_P(1:3,1:3);
    pose_init.T = best_P(1:3,4);
    
    p = pts_data*pose_init.R' + repmat(pose_init.T(:)',[size(pts_data,1),1]);
    figure; myplot3(pts_model,'b.'); hold on; myplot3(p,'r.'); hold off; axis equal; view3d
end

%%

pose = pose_init;
pose_ident.R = eye(3);
pose_ident.T = [0;0;0];
for res = res_range
    [~,world_to_vol,dist_inds] = create_dist_xform(pts_model, res, 10);

    P = [pose.R,pose.T(:)];
    pts = [pts_data,ones(size(pts_data,1),1)]*P';
    pts = decimate_points(pts,res);
    pts = [pts,ones(size(pts,1),1)]*world_to_vol';
    pts = pts(:,1:3);
    result = optimize_pose_with_directions(dist_inds,pts,pose_ident);
    
    P = [result.pose.R,result.pose.T(:);0,0,0,1];
    P = world_to_vol\P*world_to_vol*[pose.R,pose.T(:);0,0,0,1];
    pose.R = P(1:3,1:3);
    pose.T = P(1:3,4);
end

%%
if (false)
[dist,world_to_vol] = create_dist_xform(pts_model, res(end), 10);

P = [pose.R,pose.T(:)];
pts = [pts_data,ones(size(pts_data,1),1)]*P';
pts = decimate_points(pts,res);
pts = [pts,ones(size(pts,1),1)]*world_to_vol';
pts = pts(:,1:3);
result = optimize_pose(dist,pts,pose_ident);

P = [result.pose.R,result.pose.T(:);0,0,0,1];
P = world_to_vol\P*world_to_vol*[pose.R,pose.T(:);0,0,0,1];
pose.R = P(1:3,1:3);
pose.T = P(1:3,4);
end

%%
result.pose = pose;