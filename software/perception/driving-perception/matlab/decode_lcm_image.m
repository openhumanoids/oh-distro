function [img, timestamp] = decode_lcm_image(bc_img)

timestamp = bc_img.utime;

width =  bc_img.width;
height =  bc_img.height;

% img = zeros(width* height, 3);

size(bc_img.data)
img = reshape_image(uint8(bc_img.data), width, height);
size(img)
% img = bc_img.data;


%%Read image and then put [img, timestamp] in to the output