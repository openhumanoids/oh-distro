function log_data = read_log_data(logfile)

lcmlog = lcm.logging.Log(logfile,'r');
scans = {};
disparities = {};
poses = {};
imgs = {};
while (true)
    try
        event = lcmlog.readNext();
    catch ex
        break;
    end
    channel = event.channel;
    if (strcmp(channel,'SCAN'))
        scan = decode_lcm_lidar(bot_core.planar_lidar_t(event.data));
        scan.timestamp = int64(scan.utime);
        scans{end+1} = scan;
    elseif (strcmp(channel,'CAMERA'))
        obj = multisense.images_t(event.data);
        ind = (obj.image_types == obj.DISPARITY) | (obj.image_types == obj.DISPARITY_ZIPPED);
        img = obj.images(ind);
        data = img.data;
        if (obj.image_types(ind) == obj.DISPARITY_ZIPPED)
            data = zlibdecode(data);
        end
        data = typecast(data,'int16');
        img = reshape(data,[img.width,img.height])';
        disparity = single(img)/16;
        disparities{end+1}.img = disparity;
        disparities{end}.timestamp = int64(obj.images(ind).utime);
        
        ind = obj.image_types == obj.LEFT;
        imgs{end+1} = decode_lcm_image(obj.images(ind));
    elseif (strcmp(channel,'PRE_SPINDLE_TO_POST_SPINDLE'))
        obj = bot_core.rigid_transform_t(event.data);
        P = inv([quat2rot(obj.quat), obj.trans(:);0,0,0,1]);
        pose.R = P(1:3,1:3);
        pose.T = P(1:3,4);
        pose.timestamp = int64(obj.utime);
        poses{end+1} = pose;
    end
end
scans = cell2mat(scans);
log_data.disparities = cell2mat(disparities);
log_data.imgs = cell2mat(imgs);
log_data.poses = cell2mat(poses);

% augment scans
for i = 1:numel(scans)
    s = scans(i);
    thetas = s.theta_min + s.theta_step*(0:numel(s.ranges)-1);
    scans(i).thetas = thetas(:);
    scans(i).xy = [s.ranges(:).*cos(thetas(:)), s.ranges(:).*sin(thetas(:))];
end
log_data.scans = scans;
