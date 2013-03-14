close all
ranges= load('/tmp/depths.txt')
%ranges = flipud(ranges)

imagesc(ranges,[0 1])
%imagesc(ranges)

%mesh(ranges)



colorbar