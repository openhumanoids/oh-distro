%% for simple road extraction

%%
%setup_lcm
addpath utils
javaaddpath /usr/local/share/java/lcm.jar
javaaddpath /home/sachih/drc/software/build/share/java/lcmtypes_bot2-core.jar
%lc = lcm.lcm.LCM.getSingleton();


%% read data
lcmlog = lcm.logging.Log('/home/sachih/lcm_logs/driving_task/lcmlog-2013-03-27.01','r');
counter_image = 0;
counter_pose = 0;
imgs = repmat(struct,[0,1]);
poses = repmat(struct,[0,1]);

while (true)
    try
        event = lcmlog.readNext();
    catch ex
        break;
    end
    channel = event.channel;
    if (strcmp(channel,'CAMERALEFT'))
        counter_image = counter_image+1;
        %sample every 10th image
        if (mod(counter_image,10)==0)
            img_data = bot_core.image_t(event.data);
            [img,timestamp] = decode_lcm_image(bot_core.image_t(event.data));
            imgs(numel(imgs)+1).img_raw = img;
            imgs(numel(imgs)).timestamp = timestamp;
            fprintf('got image %d\n', numel(imgs));            
            if(numel(imgs)==10)
               break;
            end
        end
    elseif (strcmp(channel,'POSE_HEAD'))
        counter_pose = counter_pose+1;
        pose = bot_core.pose_t(event.data);
        poses(counter_pose).quat = pose.orientation;
        poses(counter_pose).T = pose.pos;
        poses(counter_pose).timestamp = int64(pose.utime);
    end
end
fprintf('found %d images and %d poses\n', counter_image, counter_pose);

%% sync poses
t_poses = cat(1,poses.timestamp);
for i = 1:numel(imgs)
    t_im = imgs(i).timestamp;
    d = t_im-t_poses;
    ind = find(d>=0, 1, 'last');
    frac = double(d(ind))/double((t_poses(ind+1)-t_poses(ind)));
    T_interp = (1-frac)*poses(ind).T + frac*poses(ind+1).T;
    q_interp = (1-frac)*poses(ind).quat + frac*poses(ind+1).quat;
    R_interp = quat2rot(q_interp);
    imgs(i).R = R_interp;
    imgs(i).T = T_interp;
end

%% simple hsv road filter
for i = 1:numel(imgs)
    hsv = rgb2hsv(imgs(i).img_raw);
    m = hsv(:,:,2)<0.05 & hsv(:,:,3)>0.1 & hsv(:,:,3)<0.4;
    imgs(i).msk = m;
    fprintf('masked %d/%d\n', i, numel(imgs));
end

%% draw initial road mask
figure;
for i = 1:numel(imgs)
    foo = imgs(i).img_raw;
    foo(:,:,1) = uint8(0.75*foo(:,:,1)) + uint8(0.25*imgs(i).msk*255);
    %plothot(foo); drawnow;
end

%% draw head poses as read from log
figure;
for i = 1:numel(imgs)
    hold on; plot_pose(imgs(i).R,imgs(i).T,100,'');
end
axis equal; view3d

%% try to find road edges

% note: this assumes that image mask exists for rollbar and other car
% elements
% msk = false(size(imgs(1).img_raw));
% figure; msk = roipoly(imgs(1).msk);

cam = StereoCamera();
left_cam = cam.getLeft;
f = fspecial('gaussian',[9,1],1);
plane = [0,0,1,0]';
P_cam = left_cam.K*left_cam.R'*[eye(3),-left_cam.T(:)];
for i = 1:numel(imgs)

    % form homography from image to road
    P = P_cam/[imgs(i).R,imgs(i).T(:);0,0,0,1];
    H_local_to_img = P(:,[1,2,4]);
    H_img_to_local = inv(H_local_to_img);

    % morphology to form one solid road polygon (this could use
    % a good deal of improvement)
    
    % remove visible car elements from mask
    m = imgs(i).msk & ~msk;

    % remove small connected components
    m = bwareaopen(m,10000);

    % remove small holes
    m = imclose(m,ones(5,5));
    
    % form convex polygon
    lab = bwlabel(m);
    lab(lab>0) = 1;
    props = regionprops(lab,'ConvexHull');
    m = poly2mask(props.ConvexHull(:,1),props.ConvexHull(:,2),size(m,1),size(m,2));
    m(:,[1,end]) = false;
    imgs(i).msk2 = m;

    % find edges of road in image space
    e = edge(m,'canny');
    [y,x] = find(e);
    pts = [x,y,ones(size(x))]*H_img_to_local';
    pts = pts(:,1:2)./pts(:,[3,3]);
    m_smoothed = myconv2(f,f,double(m));
    [gx,gy] = gradient(m_smoothed);
    gx = gx(e>0);
    gy = gy(e>0);
    gmag = hypot(gx,gy);
    ngx = gx./gmag;
    ngy = gy./gmag;
    pts1 = [x+ngx,y+ngy,ones(size(x))]*H_img_to_local';
    pts1 = pts1(:,1:2)./pts1(:,[3,3]);
    norms = pts1-pts;
    norms = norms./repmat(sqrt(sum(norms.^2,2)),[1,2]);

    % form grid map (bounds and resolution currently hardcoded)
    
    % project road mask onto xy plane via homography
    gridmap = imtransform(imgs(i).msk2,maketform('projective',H_img_to_local'),...
        'bilinear','XData',[0,150],'YData',[-10,60],'XYScale',0.25);

    % form distance transform for road surface
    dist_xform_road = bwdist(gridmap==0);

    % form distance transform for distance from current position
    cur_xy = (imgs(i).T(1:2)' - [0,-10])/0.25;
    positionmap = false(size(gridmap));
    positionmap(round(cur_xy(2)),round(cur_xy(1))) = true;
    dist_xform_range = bwdist(positionmap);

    % composite inverse cost function
    dist_xform = dist_xform_road.*dist_xform_range;

    % try to detect ridges
    ridges = find_ridges(dist_xform);
    [y,x] = find(ridges);
    x = x*.25 + 0;
    y = y*.25 + -10;

    % project ridges back into image
    img_pts = [x,y,ones(size(x))]*H_local_to_img';
    img_pts = img_pts(:,1:2)./img_pts(:,[3,3]);
    
    % draw stuff
    figure(22);
    subplot(2,2,1);
    plothot(dist_xform_road); colorbar off;
    hold on; myplot(cur_xy,'g.'); hold off;
    axis xy;
    subplot(2,2,2);
    plothot(dist_xform_range); colorbar off;
    hold on; myplot(cur_xy,'g.'); hold off;
    axis xy;
    subplot(2,2,3);
    imshow(imgs(i).img_raw);
    hold on; myplot(img_pts,'g.','markersize',10); hold off;
    %myplot(pts,'r.'); axis equal; axis([0,150,-10,100]);
    subplot(2,2,4);
    plothot(dist_xform); colorbar off;
    hold on; myplot(cur_xy,'g.'); hold off;
    axis xy;
    drawnow;
    
    
    fprintf('edges %d/%d\n', i, numel(imgs));
    pause(0.5);
end

%% todo: cost map
