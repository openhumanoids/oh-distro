function [img] = reshape_image(raw_data, width, height)
row_stride = width * 3;
r = reshape(raw_data,row_stride,height)';
size(r)

img = zeros(height, width, 3);

for i=1:height
   row = r(i,:);
   row_array = reshape(row,3,width);
   img(i,:,:) = row_array';
end

imshow(img);
