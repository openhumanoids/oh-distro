clear all;
roll =0;   pitch = 0;  yaw =-90*(pi/180);
q0 = rpy2quat(  [roll, pitch, yaw])';

roll =0;   pitch = 0;  yaw =140*(pi/180);
q1 = rpy2quat(  [roll, pitch, yaw])';


s=[0:0.01:1];
val = [];angvel_foh=[];
rpy= [];
for i=1:length(s),
    val(:,i)=quat_foh([q0' q1' -q0'],s(i));
    rpy(:,i)=quat2rpy(val(:,i))'*(180/pi);
    disp(rpy(3,i))
end    
for i=1:length(s),    
    val2(:,i)=quat_spline([q0' q1' -q0'],s(i));
    rpy2(:,i)=quat2rpy(val2(:,i))'*(180/pi);
    disp(rpy2(3,i))
end


figure;
plot(rpy(3,:),'r-');
hold on;
%plot(rpy(1,:),'g-');plot(rpy(2,:),'b-');plot(rpy2(1,:),'g:.');plot(rpy2(2,:),'b:.');
plot(rpy2(3,:),'r:.');
hold off;