close all
ranges= load('/tmp/ranges.txt')
%ranges = flipud(ranges)

%imagesc(ranges,[0 5])
imagesc(ranges)
colorbar