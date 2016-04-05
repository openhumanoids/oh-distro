function out = decode_lcm_image(in)

out.img = [];
w = in.width;
h = in.height;
s = in.row_stride;
format = in.pixelformat;
raw = typecast(in.data,'uint8');

out.timestamp = int64(in.utime);
if (format == in.PIXEL_FORMAT_GRAY)
    out.img = reshape(raw,[s,h]);
    out.img = out.img(1:w,:)';
elseif (format == in.PIXEL_FORMAT_RGB)
    tmp = reshape(raw,[s,h]);
    out.img = zeros(h,w,3,'uint8');
    for k = 1:3
        out.img(:,:,k) = tmp(k:3:3*(w-1)+k,:)';
    end
elseif (format == in.PIXEL_FORMAT_MJPEG)
    fp = fopen('/tmp/foo.jpg','w');
    fwrite(fp,raw);
    fclose(fp);
    try
        out.img = imread('/tmp/foo.jpg');
    catch ex
        out.img = [];
    end
else
    fprintf('Cannot currently handle this pixel format.\n');    
end
