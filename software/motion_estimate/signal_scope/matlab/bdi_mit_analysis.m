% Script to analyse the contents of ATLAS_STATE lcm message
%
%bot-log2mat manip02-lcmlog-2014-04-21-15-51-robot  -c "ATLAS_STATE" -o log_out.mat -i "ATLAS_STATE_EXTRA"
% mat file 20x smaller than entire lcmlog
%
% 1      - unix time (nsec)
% 2      - num joints (28)
% 3-30   - joint_position
% 31-58  - joint_velocity
% 59-86  - joint_effort
% 87-104 - force_torque sensors
% 105    - time from zero (sec)
%
% positions, velocity and effort ordered by AtlasJointId 
% (from % AtlasControlTypes.h)
% back, neck, l_leg, r_leg, l_arm, r_arm

%load('~/Desktop/bdi/ladder.mat')
load('ladder.mat')

% convert time to seconds
t = (ATLAS_STATE(1:end,1) - ATLAS_STATE(1,1) )*1E-6;

% right arm:
figure;
plot(t,  ATLAS_STATE(1:end,81:86))
xlabel('time [sec]')
ylabel('joint effort')
title('Right Arm')

% right arm:
figure;
plot(t  ,  ATLAS_STATE(1:end,75:80))
xlabel('time [sec]')
ylabel('joint effort')
title('Left Arm')


figure
plot( ATLAS_STATE(1:end,25:30) ,  ATLAS_STATE(1:end,81:86),'.')
xlabel('joint angle')
ylabel('joint effort')
title('Right Arm')
