function [active_stack] = make_activity(h,data_path,task,init_utime)
%close all, clear all

d = load( [data_path '/' task '/task_anatomy_plans.txt'])


d(:,1) = (d(:,1) - init_utime)*1E-6
d(:,2) = d(:,2)*1E-6

%plot(d(:,1),d(:,2),'.r')

active_binary = zeros(4000,1);
plan_binary = zeros(4000,1);
active_moving = zeros(4000,1);
plan_moving = zeros(4000,1);

for i=1:size(d,1) 
  active_tics = round(d(i,1)) : round(  sum(d(i,:)));
  d(i,:)
  %keyboard
  active_binary(active_tics,1) =1;
  
  plan_tics = round(d(i,1)) ;  
  plan_binary( plan_tics,1) = 1;
end


win=30;
skip = 30
j=1
for i=win:skip:size(active_binary,1)
  a_m = mean(active_binary(i-win+1:i));
  p_m = mean(plan_binary(i-win+1:i))*60; % plans per min
  active_moving(i) = a_m;
  plan_moving(i) = p_m;
  
  active_moving_sample(j) = a_m;
  plan_moving_sample(j) = p_m;
  j=j+1;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%
walking_binary = zeros(4000,1);
walk = [1/60,0.7;...
1.23, 1.93;...
8.22,	8.78;...
14.88,	15.40]*60
for i=1:size(walk,1) 
  % slightly different:
  active_tics = round(walk(i,1)) : round(  sum(walk(i,2)))
  
  walking_binary(active_tics,1) =1;
  
end

j=1
walking_moving_sample = zeros(30,1)
for i=win:skip:size(walking_binary,1)
  w_m = mean(walking_binary(i-win+1:i));
  walking_moving(i) = w_m;
  
  
  walking_moving_sample(j) = w_m;
  j=j+1;
end


% figure; plot(active_moving)
% figure; plot(plan_moving)
% 
% figure; plot(active_moving_sample)
% figure; plot(plan_moving_sample)


%figure
%bar(active_moving_sample)
%hold on



active_stack = [active_moving_sample; walking_moving_sample'];
t= 0.5*(1:size(active_stack,2)) -0.25
%figure

h1=bar( t', active_stack'*100  , 1,'stack')
hold on



axis([0 50 0 100])


colors = [  31/255.0, 120/255.0, 180/255.0;...
  106/255.0, 61/255.0, 154/255.0;...
  178/255.0, 223/255.0, 138/255.0;...
  166/255.0, 206/255.0, 227/255.0]

P=findobj(gca,'type','patch');
C=['w','k','m'];%,'g',...); % make a colors list 
for n=1:length(P) 
set(P(n),'facecolor',colors(n,:));
end

%ylabel('Activity %')
%ylabel('Blue Walking, Purple Manip. ')


tlabel(1) = text(-3.5, 50,'Activity %');%,'FontSize',18)
set( tlabel,'Rotation',90,'HorizontalAlignment','center');
tlabel(1) = text(-2.5, 20,'Walking','Color',colors(1,:))
set( tlabel,'Rotation',90,'HorizontalAlignment','center');
tlabel(1) = text(-2.5, 75,'Manip.','Color',colors(2,:))
set( tlabel,'Rotation',90,'HorizontalAlignment','center');



axis([0 30 0 100])

set(h1,'EdgeColor','k')