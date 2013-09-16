%%
walls_cloud = load('/home/antone/temp/walls_cloud.txt');
walls_stereo = load('/home/antone/temp/walls_stereo.txt');

%%
ref_pts = walls_cloud(:,1:3);
cur_pts = walls_stereo(:,1:3);

max_r = 3;
min_r = 1;
ref_r = sqrt(sum(ref_pts.^2,2));
cur_r = sqrt(sum(cur_pts.^2,2));
ref_pts(ref_r>max_r | ref_r<min_r,:) = [];
cur_pts(cur_r>max_r | cur_r<min_r,:) = [];

%%
p = ref_pts;
planes_ref = struct;
counter = 1;
while (true)
    result = ransac_plane(p,0.1);
    fprintf('inliers %d/%d\n', sum(result.inliers),numel(result.inliers));
    if (sum(result.inliers)<10000)
        break;
    end
    planes_ref(counter).plane = result.sol;
    planes_ref(counter).pts = p(result.inliers,:);
    p = p(~result.inliers,:);
    counter = counter+1;
end

best_score = 1e10;
for i = 1:numel(planes_ref)-2
    for j = i+1:numel(planes_ref)-1
        for k = j+1:numel(planes_ref)
            R = [planes_ref(i).plane(1:3),planes_ref(j).plane(1:3),planes_ref(k).plane(1:3)];
            score = norm(eye(3)-R'*R);
            if (score < best_score)
                best_score = score;
                best_triple = [i,j,k];
            end
        end
    end
end

planes = cat(2,planes_ref(best_triple).plane)';
ind = planes(:,4)<0;
planes(ind,:) = -planes(ind,:);
normals = planes(:,1:3);
[~,ind_z] = min(normals(:,1));
z_dir = normals(ind_z,:);
[~,ind_x] = max(normals(:,2));
x_dir = normals(ind_x,:);
[~,ind_y] = min(normals(:,2));
y_dir = normals(ind_y,:);
corner = null(planes([ind_x,ind_y,ind_z],:));
corner = corner(1:3)/corner(4);
T = corner;
R = [x_dir(:),y_dir(:),z_dir(:)];
[u,~,v] = svd(R);
R = u*v';
P = [R,T(:);0,0,0,1];
P_lidar2world = inv(P);

%%
p = cur_pts;
planes_cur = struct;
counter = 1;
while (true)
    result = ransac_plane(p,0.1);
    fprintf('inliers %d/%d\n', sum(result.inliers),numel(result.inliers));
    if (sum(result.inliers)<10000)
        break;
    end
    planes_cur(counter).plane = result.sol;
    planes_cur(counter).pts = p(result.inliers,:);
    p = p(~result.inliers,:);
    counter = counter+1;
end

best_score = 1e10;
for i = 1:numel(planes_cur)-2
    for j = i+1:numel(planes_cur)-1
        for k = j+1:numel(planes_cur)
            R = [planes_cur(i).plane(1:3),planes_cur(j).plane(1:3),planes_cur(k).plane(1:3)];
            score = norm(eye(3)-R'*R);
            if (score < best_score)
                best_score = score;
                best_triple = [i,j,k];
            end
        end
    end
end

planes = cat(2,planes_cur(best_triple).plane)';
ind = planes(:,4)<0;
planes(ind,:) = -planes(ind,:);
normals = planes(:,1:3);
[~,ind_z] = min(normals(:,2));
z_dir = normals(ind_z,:);
[~,ind_x] = min(normals(:,1));
x_dir = normals(ind_x,:);
[~,ind_y] = max(normals(:,1));
y_dir = normals(ind_y,:);
corner = null(planes([ind_x,ind_y,ind_z],:));
corner = corner(1:3)/corner(4);
T = corner;
R = [x_dir(:),y_dir(:),z_dir(:)];
[u,~,v] = svd(R);
R = u*v';
P = [R,T(:);0,0,0,1];
P_cam2world = inv(P);

%%
P_cam2lidar = inv(P_lidar2world)*P_cam2world;

%%
p = [cur_pts,ones(size(cur_pts,1),1)]*P_cam2lidar';
figure;
hold on;
myplot3(ref_pts,'b.');
myplot3(p,'r.');
hold off;
axis equal;
view3d;


%% optimize distance transform distances
res = 0.01;
[dist,world_to_vol] = create_dist_xform(ref_pts, res, 10);

P = P_cam2lidar(1:3,:);
pts = [cur_pts,ones(size(cur_pts,1),1)]*P';
pts = decimate_points(pts,res);
pts = [pts,ones(size(pts,1),1)]*world_to_vol';
pts = pts(:,1:3);
result = optimize_pose(dist,pts,pose_ident);

P = [result.pose.R,result.pose.T(:);0,0,0,1];
P = world_to_vol\P*world_to_vol*[pose.R,pose.T(:);0,0,0,1];
pose.R = P(1:3,1:3);
pose.T = P(1:3,4);
P_cam2lidar_final = [pose.R,pose.T(:);0,0,0,1];


%% OR, iterate closest point transform
res = 0.01;
pose_ident.R = eye(3);
pose_ident.T = [0;0;0];
pose_init.R = P_cam2lidar(1:3,1:3);
pose_init.T = P_cam2lidar(1:3,4);
[~,world_to_vol,dist_inds] = create_dist_xform(ref_pts, res, 10);

P = [pose_init.R,pose_init.T(:)];
pts = [cur_pts,ones(size(cur_pts,1),1)]*P';
pts = decimate_points(pts,res);
pts = [pts,ones(size(pts,1),1)]*world_to_vol';
pts = pts(:,1:3);
result = optimize_pose_with_directions(dist_inds,pts,pose_ident);

P = [result.pose.R,result.pose.T(:);0,0,0,1];
P = world_to_vol\P*world_to_vol*[pose_init.R,pose_init.T(:);0,0,0,1];
pose.R = P(1:3,1:3);
pose.T = P(1:3,4);
P_cam2lidar_final = [pose.R,pose.T(:);0,0,0,1];


%% load table data
table_cloud = load('/home/antone/temp/table_cloud.txt');
table_stereo = load('/home/antone/temp/table_stereo.txt');

%%
ref_pts = table_cloud(:,1:3);
cur_pts = table_stereo(:,1:3);

max_r = 2;
min_r = 0.5;
ref_r = sqrt(sum(ref_pts.^2,2));
cur_r = sqrt(sum(cur_pts.^2,2));
ref_pts(ref_r>max_r | ref_r<min_r,:) = [];
cur_pts(cur_r>max_r | cur_r<min_r,:) = [];

%%
res = 0.01;
pose_init.R = P_cam2lidar_final(1:3,1:3);
pose_init.T = P_cam2lidar_final(1:3,4);
[~,world_to_vol,dist_inds] = create_dist_xform(ref_pts, res, 10);

P = [pose_init.R,pose_init.T(:)];
pts = [cur_pts,ones(size(cur_pts,1),1)]*P';
pts = decimate_points(pts,res);
pts = [pts,ones(size(pts,1),1)]*world_to_vol';
pts = pts(:,1:3);
result = optimize_pose_with_directions(dist_inds,pts,pose_ident);

P = [result.pose.R,result.pose.T(:);0,0,0,1];
P = world_to_vol\P*world_to_vol*[pose_init.R,pose_init.T(:);0,0,0,1];
pose.R = P(1:3,1:3);
pose.T = P(1:3,4);
P_cam2lidar_final2 = [pose.R,pose.T(:);0,0,0,1];


%%
P_lidar2cam = inv(P_cam2lidar_final2)
q_final = rot2quat(P_lidar2cam(1:3,1:3));
pos_final = P_lidar2cam(1:3,4);
q_final(:)'
pos_final(:)'
