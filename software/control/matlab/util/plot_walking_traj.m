close all;
clear all;
load('fast-walking-03.mat');

heel = [-0.082,-0.082;-0.0624435,0.0624435];
toe =  [0.178,  0.178;-0.0624435,0.0624435];

colorstr = 'krgbmykrgbmy';

n_right_clusters=5;
n_left_clusters=5;

left_clusters = clusterdata(lfoot_knots(1,:)',n_left_clusters);
right_clusters = clusterdata(rfoot_knots(1,:)',n_right_clusters);
% 
% figure(1);
% hold on;
% for i=1:n_left_clusters
%   plot(lfoot_knots(1,left_clusters==i),lfoot_knots(2,left_clusters==i),strcat(colorstr(i),'.'));
% end
% for i=1:n_right_clusters
%   plot(rfoot_knots(1,right_clusters==i),rfoot_knots(2,right_clusters==i),strcat(colorstr(i),'.'));
% end
% hold off;

figure(2);
hold on;
for i=1:n_left_clusters
  x=mean(lfoot_knots(1,left_clusters==i));
  y=mean(lfoot_knots(2,left_clusters==i));
%   plot(x,y,'go');

  xs = x+[heel(1,1),heel(1,2),toe(1,2),toe(1,1),heel(1,1)];
  ys = y+[heel(2,1),heel(2,2),toe(2,2),toe(2,1),heel(2,1)];
  plot(xs,ys,'k-');
end

for i=1:n_right_clusters
  x=mean(rfoot_knots(1,right_clusters==i));
  y=mean(rfoot_knots(2,right_clusters==i));
%   plot(x,y,'go');
  
  xs = x+[heel(1,1),heel(1,2),toe(1,2),toe(1,1),heel(1,1)];
  ys = y+[heel(2,1),heel(2,2),toe(2,2),toe(2,1),heel(2,1)];
  plot(xs,ys,'k-');
end

sm_win=100;
plot(smooth(cop_knots(1,:),sm_win),smooth(cop_knots(2,:),sm_win),'b', ...
    com_knots(1,:),com_knots(2,:),'r','LineWidth',2); 
hold off;
axis equal;