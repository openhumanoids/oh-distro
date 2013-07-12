%NOTEST

options.floating = true;
options.dt = 0.001;

r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);

ts = 0:0.001:.5;

load drake_xtraj_0.mat;
dxtraj = xtraj;
load gazebo_xtraj_0.mat;
gxtraj = xtraj;
% load gazebo_xtraj_sideways_tip_TRUE.mat;
% gxtraj_true = xtraj;

drake_xtraj = zeros(getNumStates(r),length(ts));
gazebo_xtraj = zeros(getNumStates(r),length(ts));
% gazebo_xtraj_true = zeros(getNumStates(r),length(ts));
gazebo_xtraj_trans = zeros(getNumStates(r),length(ts));
for i=1:length(ts)
  t=ts(i);
  drake_xtraj(:,i) = dxtraj.eval(t);
  gazebo_xtraj(:,i) = gxtraj.eval(t);
%   gazebo_xtraj_true(:,i) = gxtraj_true.eval(t);
  gazebo_xtraj_trans(:,i) = gxtraj.eval(t);
%   R_delta = rpy2rotmat(gazebo_xtraj(34+(4:6),i));
  R = rpy2rotmat(gazebo_xtraj(4:6,i));
%   v=rotmat2rpy(R_delta*R);
%   vx=rotmat2rpy(rotx(gazebo_xtraj(34+(4)))*R);
%   vy=rotmat2rpy(roty(gazebo_xtraj(34+(5)))*R);
%   vz=rotmat2rpy(rotz(gazebo_xtraj(34+(6)))*R);
%   vx=gazebo_xtraj(34+(4));
%   vy=gazebo_xtraj(34+(5));
%   vz=gazebo_xtraj(34+(6));
%   S = [0 -vz vy; vz 0 -vx; -vy vx 0];
%   s = [vx;vy;vz];
%   V=cross(s,R);
  gazebo_xtraj_trans(34+(4:6),i) = R'*gazebo_xtraj_trans(34+(4:6),i);
end

% for i=[1:6, 34+(1:6)]%1:getNumStates(r)
%   figure(i);
%   hold on;
%   plot(ts,drake_xtraj(i,:),'g');
%   plot(ts,gazebo_xtraj(i,:),'r');
%   title(r.getStateFrame.coordinates(i));
%   hold off;
% end

figure(1);
plot(ts,drake_xtraj(34+4,:),'g');
hold on;
plot(ts,gazebo_xtraj(34+4,:),'b');
hold off;

figure(2);
plot(ts,drake_xtraj(34+5,:),'g');
hold on;
plot(ts,gazebo_xtraj(34+5,:),'b');
hold off;

figure(3);
plot(ts,drake_xtraj(34+4,:),'g');
hold on;
plot(ts,gazebo_xtraj_trans(34+4,:),'b');
hold off;

figure(4);
plot(ts,drake_xtraj(34+5,:),'g');
hold on;
plot(ts,gazebo_xtraj_trans(34+5,:),'b');
hold off;

% figure(1);
% plot(ts,drake_xtraj(4,:),'g');
% hold on;
% plot(ts,gazebo_xtraj(4,:),'r');
% plot(ts,gazebo_xtraj_true(4,:),'b');
% legend('drake roll','gazebo roll','gazebo true roll');
% hold off;
% 
% figure(2);
% plot(ts,drake_xtraj(5,:),'g');
% hold on;
% plot(ts,gazebo_xtraj(5,:),'r');
% plot(ts,gazebo_xtraj_true(5,:),'b');
% legend('drake pitch','gazebo pitch','gazebo true pitch');
% hold off;
% 
% figure(3);
% plot(ts(1:end-1),drake_xtraj(34+4,1:end-1),'g');
% hold on;
% plot(ts(1:end-1),(drake_xtraj(4,2:end)-drake_xtraj(4,1:end-1))/0.001,'b');
% legend('drake roll dot','drake roll finite difference');
% hold off;
% 
% figure(4);
% plot(ts(1:end-1),drake_xtraj(34+5,1:end-1),'g');
% hold on;
% plot(ts(1:end-1),(drake_xtraj(5,2:end)-drake_xtraj(5,1:end-1))/0.001,'b');
% legend('drake pitch dot','drake pitch finite difference');
% hold off;
% 
% figure(5);
% plot(ts(1:end-1),gazebo_xtraj(34+4,1:end-1),'r');
% hold on;
%  plot(ts(1:end-1),gazebo_xtraj_true(34+4,1:end-1),'g');
% plot(ts(1:end-1),(drake_xtraj(4,2:end)-drake_xtraj(4,1:end-1))/0.001,'b');
% legend('gazebo roll dot','gazebo roll dot true','drake roll finite difference');
% hold off;
% 
% figure(6);
% plot(ts(1:end-1),gazebo_xtraj(34+5,1:end-1),'r');
% hold on;
% plot(ts(1:end-1),gazebo_xtraj_true(34+5,1:end-1),'g');
% plot(ts(1:end-1),(drake_xtraj(5,2:end)-drake_xtraj(5,1:end-1))/0.001,'b');
% legend('gazebo pitch dot','gazebo pitch dot true','drake pitch finite difference');
% hold off;
