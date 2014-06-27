%%
clear pts
pts{1} = load('/home/antone/temp/head_data/walking_cloud_before.txt');
pts{2} = load('/home/antone/temp/head_data/walking_cloud_after.txt');
pts{3} = load('/home/antone/temp/head_data/settling_cloud_before.txt');
pts{4} = load('/home/antone/temp/head_data/settling_cloud_after.txt');
pts{5} = load('/home/antone/temp/head_data/standing_cloud_before.txt');
pts{6} = load('/home/antone/temp/head_data/standing_cloud_after.txt');


% compare stationary scan with walking scan
figure, hold on;
myplot3(pts{1},'b.');
myplot3(pts{5},'r.');
hold off; axis equal; view3d on
title('walking (blue), stationary (red)');

figure, hold on;
myplot3(pts{1},'b.');
myplot3(pts{4},'r.');
hold off; axis equal; view3d on
title('walking (blue), settling (red)');

d1 = pts{1}-pts{2};
d1 = sqrt(sum(d1.^2,2));
d2 = pts{3}-pts{4};
d2 = sqrt(sum(d2.^2,2));
d3 = pts{5}-pts{6};
d3 = sqrt(sum(d3.^2,2));
stats1.mean = mean(d1);
stats1.std = std(d1);
stats1.max = max(d1);
stats2.mean = mean(d2);
stats2.std = std(d2);
stats2.max = max(d2);
stats3.mean = mean(d3);
stats3.std = std(d3);
stats3.max = max(d3);

%%
p1 = load('/home/antone/temp/head_data/standing_cloud_before.txt');
p2 = load('/home/antone/temp/head_data/settling_cloud_before.txt');
p3 = load('/home/antone/temp/head_data/registered.txt');
d = p2-p3;
figure, myplot3(p1,'b.'); hold on; myplot3(p3,'r.'); hold off; axis equal; view3d on
dists = sqrt(sum(d.^2,2));
figure, hist(dists,100);
median(dists)
