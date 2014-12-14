function main()
% randomly sample points in a triangle to specified density
close all

pts_per_msquared = 10

% triangles:
p0 = [ 0, 0, 0]
p1 = [ 0, 10, 5]
p2 = [ 7, 9, 12]

a = areaOfTriange(p0,p1,p2)
n_pts = a*pts_per_msquared;

for i=1:n_pts
  pts(i,:)=getPointInTriangle(p0,p1,p2);
end


figure; hold on; axis equal
plot3( [p0(1) p1(1) p2(1)  p0(1)] , [p0(2) p1(2) p2(2)  p0(2)]  ,...
         [p0(3) p1(3) p2(3)  p0(3)] )

plot3(pts(:,1), pts(:,2),pts(:,3),'r.')
axis([0 5 0 5])

function a = areaOfTriange(p0,p1,p2)
% heron's formula
d01 =  sqrt( sum( (p0 - p1).^2));
d02 =  sqrt( sum( (p0 - p2).^2));
d12 =  sqrt( sum( (p1 - p2).^2));
p = ( d01 + d02 + d12 ) /2;
a = sqrt(p*(p- d01)*(p- d02)*(p- d12));


function tript=getPointInTriangle(p0,p1,p2)
% from wykobi library
a = rand(1,1);
b = rand(1,1);
if ((a + b) > 1)
         a = 1 - a;
         b = 1 - b;
end
c = (1 - a - b);
tript = [ (p0(1) .* a) + (p1(1) .* b) + (p2(1) .* c) , ...
  (p0(2) .* a) + (p1(2) .* b) + (p2(2) .* c) , ...
  (p0(3) .* a) + (p1(3) .* b) + (p2(3) .* c) ];