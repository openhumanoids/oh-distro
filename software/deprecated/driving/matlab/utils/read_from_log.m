function out = read_from_log(filename, channels, msgtypes, log_interval)

if (nargin < 4)
    log_interval = [0,1];
end

if (log_interval(1) < 0)
    error('Interval minimum must be non-negative');
elseif (log_interval(2) > 1)
    error('Interval maximum must be greater than 1');
elseif (log_interval(2) < log_interval(1))
    error('Interval maximum must be greater than interval minimum');
end

if (~iscell(channels) || ~iscell(msgtypes))
    error('Channels and message types must be cell arrays of strings.');
end

if (numel(channels) ~= numel(msgtypes))
    error('Number of channels must equal number of message types');
end

try
    log = lcm.logging.Log(filename,'r');
catch
    error('Cannot open log file %s', filename);
end

try
    log.seekPositionFraction(log_interval(1));
catch
    error('Cannot seek to position %f', log_interval(1));
end

for i = 1:numel(channels)
    out_list.(lower(channels{i})) = {};
    which_chan_map.(channels{i}) = i;
end

check_position_interval = 100;
check_position_counter = 0;
last_percent = 0;

count = 0;
while (true)
    try
        if (mod(check_position_counter,check_position_interval)==0)
            position = log.getPositionFraction();
            pct_done = 100*(position-log_interval(1))/(log_interval(2)-log_interval(1));
            if (round(pct_done) > last_percent)
                last_percent = round(pct_done);
                fprintf('%f\n', pct_done);
            end
            if (position > log_interval(2))
                break;
            end
        end
        event = log.readNext();
    catch
        break;
    end

    check_position_counter = check_position_counter+1;
    chan_str = regexprep(char(event.channel),['([ -\/\\\*\?\:\[\]\' ...
                        '{\}\<\>\=\+\"]+)'],'_');
    try
        which_chan = which_chan_map.(chan_str);
    catch
        which_chan_map.(chan_str) = -1;
        which_chan = -1;
    end
    if (which_chan < 0)
        continue;
    end
    clear s;
    bytestream = java.io.ByteArrayInputStream(event.data,0,numel(event.data));
    datastream = java.io.DataInputStream(bytestream);
    msgtype = msgtypes{which_chan};
    if (strcmpi(msgtype, 'bot_core.planar_lidar_t'))
        item = bot_core.planar_lidar_t(datastream);
        n = numel(item.ranges);
        s.t = item.utime;
        s.thetas = item.rad0:item.radstep:item.rad0+item.radstep*(n-1);
        s.ranges = item.ranges(:)';
        out_list.(lower(channels{which_chan})){end+1} = s;
    elseif (strcmpi(msgtype, 'arlcm.vicon_data_t'))
        item = arlcm.vicon_data_t(datastream);
        s.t = item.utime;
        s.bot_pose = [item.pose(1).pos(:)', item.pose(1).orientation(:)'];
        s.mast_pose = [item.pose(2).pos(:)', item.pose(2).orientation(:)'];
        s.box_pose = [item.pose(3).pos(:)', item.pose(3).orientation(:)'];
        s.pole1_pose = [item.pose(4).pos(:)', item.pose(4).orientation(:)'];
        s.pole2_pose = [item.pose(5).pos(:)', item.pose(5).orientation(:)'];
        s.pole3_pose = [item.pose(6).pos(:)', item.pose(6).orientation(:)'];
        out_list.(lower(channels{which_chan})){end+1} = s;
    elseif (strcmpi(msgtype, 'erlcm.cylinder_list_t'))
        item = erlcm.cylinder_list_t(datastream);
        s.t = item.utime;
        s.count = item.count;
        for i=1:s.count
            s.line_point{i} = item.list(i).line_point(:)';
            s.line_direction{i} = item.list(i).line_direction(:)';
            s.radius = item.list(i).radius;
        end;
        out_list.(lower(channels{which_chan})){end+1} = s;
    elseif (strcmpi(msgtype, 'erlcm.segment_list_t'))
        item = erlcm.segment_list_t(datastream);
        s.t = item.utime;
        s.count = item.no_segments;
        if (s.count==0)
            continue;
        end;
        for i=1:s.count
            seg.count = item.segments(i).no_points;
            seg.points = item.segments(i).points;
            s.seg{i} = seg;
        end;
        out_list.(lower(channels{which_chan})){end+1} = s;
    elseif (strcmpi(msgtype, 'erlcm.plane_model_t'))
        item = erlcm.plane_model_t(datastream);
        s.t = item.utime;
        s.params = item.params(:)';
        out_list.(lower(channels{which_chan})){end+1} = s;
    elseif (strcmpi(msgtype, 'bot_core.pose_t'))
        item = bot_core.pose_t(datastream);
        s.t = item.utime;
        s.position = item.pos(:)';
        s.orientation = item.orientation(:)';
        out_list.(lower(channels{which_chan})){end+1} = s;
    elseif (strcmpi(msgtype, 'bot_core.image_t'))
        item = bot_core.image_t(datastream);
        s.t = item.utime;
        s.img = rgb2gray(jpeg_decompress(item.data));
        out_list.(lower(channels{which_chan})){end+1} = s;
    end

    % count = count + 1;
    % if (count > 100)
    %     break;
    % end;
    % logdata.pcl_segment_list(1).seg{1}.points(1).xyz(:)
end

log.close();

names = fieldnames(out_list);
for i = 1:numel(names)
    for j = 1:numel(out_list.(names{i}))
        out.(names{i})(j) = out_list.(names{i}){j};
    end
end
