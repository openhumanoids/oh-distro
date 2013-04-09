width = 30;
height = 20;

f = zeros(width*height*3,1);
row_stride = width * 3;


for i = 1:height
    for j=1:width
        index = row_stride * (i-1) + 3 * (j-1) +3;
        f(index) = 1.0;                
    end
end

r = reshape(f,row_stride,height)';
size(r)

img = zeros(height, width, 3);

for i=1:height
   row = r(i,:);
%    size(row)
   row_array = reshape(row,3,width);
   img(i,:,:) = row_array';
%    img = [img; row_array'];
end
imshow(img);