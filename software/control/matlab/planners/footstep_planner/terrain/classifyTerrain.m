function [feas, heights, px2world_2x3, world2px_2x3] = classifyTerrain(heights, px2world)
%% Generate the terrain grid and classify it as safe/unsafe by a few edge detector filters

px2world(1,end) = px2world(1,end) - sum(px2world(1,1:3)); % stupid matlab 1-indexing...
px2world(2,end) = px2world(2,end) - sum(px2world(2,1:3));
resample = 2;
mag = 2^(resample-1);
heights = interp2(heights, (resample-1));
px2world = px2world * [1/mag 0 0 (1-1/mag); 0 1/mag 0 (1-1/mag ); 0 0 1 0; 0 0 0 1];

world2px = inv(px2world);
world2px_2x3 = world2px(1:2,[1,2,4]);
px2world_2x3 = px2world(1:2, [1,2,4]);

Q = imfilter(heights, [1, -1]) - (0.06/mag) > 0;
Q = Q | imfilter(heights, [-1, 1]) - (0.06/mag) > 0;
Q = Q | imfilter(heights, [1; -1]) - (0.06/mag) > 0;
Q = Q | imfilter(heights, [-1; 1]) - (0.06/mag) > 0;
Q(isnan(heights)) = 1;

feas = ~Q;

end