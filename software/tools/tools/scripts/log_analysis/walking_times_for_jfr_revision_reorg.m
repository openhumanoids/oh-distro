close all
d = csvread('/home/mfallon/data/atlas/2013-12-21-competition/analysis/walking_times_for_jfr_revision_reorg.csv')

sleep = d(:,5) == 0
plan = d(:,5) == 1
walk = d(:,5) == 2



figure
plot ( d(plan,4),  d(plan,3) ,'.')
xlabel('no. of steps')
ylabel('sec per plan')

figure
plot ( d(:,1)/60,  d(:,5) ,'.')
title('0 Sleep, 1 Plan, 2 Walk')
xlabel('time [min]')


figure
plot ( d(plan,4),  d(plan,3)./d(plan,4) ,'.')
xlabel('no. of steps')
ylabel('sec per step')

figure
vals = d(plan,4)
vals = [vals]

hist(vals,[1,2,3,4,5,6])
xlabel('number of steps')
ylabel('number of plans')
title('number of plans with a set number of steps. total steps 71. 27 plans')

sum(vals)
size(vals)

total =630+695 % 119+514+743
disp('time spent a waiting for robot/maps to settle')
dsleep = sum(d(sleep,3) )
100*dsleep/ total

disp('time spent planning')
dplan = sum(d(plan,3) )
100*dplan/ total

disp('time spent walking')
dwalk = sum(d(walk,3) )
100*dwalk/ total


%figure;bar(x)


figure
set(gcf, 'Position', [300, 100, 940, 300]);
subplot(1,2,1)
hold on
plot ( [ 2  3 4 5]' ,  [ 22.5 30.3 30 29.75 ]' ,'b-','LineWidth',2)
%plot ( [ 2.85 3.15  ]' ,  [ 30.3 30.3 ]' ,'k-','LineWidth',2)

plot ( [1  2  3 4 5 6]' ,  [ 18.5 23 27.5 32 36.5 41 ]' ,'r--','LineWidth',2)

plot ( [ (d(plan,4)-0.0)  , (d(plan,4)+0.0)]' ,  [ d(plan,3)  , d(plan,3) ]' ,'b.')
xlabel('Number of Steps')
ylabel('Seconds Taken')
axis( [0.5 6.5 0 45])
box on

subplot(1,2,2)
hist(vals,[1,2,3,4,5,6,7])
axis( [0.5 6.5 -inf 20])
xlabel('Number of Steps')
ylabel('Plans of Specific Length')
box on

dplan = d(plan,:)
dplan(:,4)


disp('time spend planning 2 steps')
idx_nstep = dplan(:,4) == 2;
mean(dplan(idx_nstep,3))

disp('time spend planning 3 steps')
idx_nstep = dplan(:,4) == 3;
mean(dplan(idx_nstep,3))

disp('time spend planning 4 steps')
idx_nstep = dplan(:,4) == 4;
mean(dplan(idx_nstep,3))

disp('time spend planning 5 steps')
idx_nstep = dplan(:,4) == 5;
mean(dplan(idx_nstep,3))


% time walk:
% 2 steps - 23 sec
% 3 steps - 27 sec
% 4 steps - 32
% 5 steps - 37.5