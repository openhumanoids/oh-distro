function out = myconv2(xfilt, yfilt, in)

if (nargin == 2)
    in = yfilt;
    yfilt = xfilt;
end

out = zeros(size(in));
for i = 1:size(in,3)
    padded = padarray(in(:,:,i), [(numel(yfilt)-1)/2, (numel(xfilt)-1)/2], 'symmetric');
    out(:,:,i) = conv2(xfilt, yfilt, padded, 'valid');
end
