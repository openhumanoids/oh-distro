
% q0=rand(1,4);
% q1=rand(1,4);
% q2=rand(1,4);

roll =0;   pitch =0;  yaw = 0.0;
q0 = angle2quat(  roll, pitch, yaw, 'ZYX' );
roll =pi;   pitch =-pi/2;  yaw = 0.0;
q1 = angle2quat(  roll, pitch, yaw, 'ZYX' );
roll =0;   pitch = pi/2;  yaw = 0.0;
q2 = angle2quat(  roll, pitch, yaw, 'ZYX' );


q0(4)=0;q1(4)=0;q2(4)=0;
q0 = q0/norm(q0);
q1 = q1/norm(q1);
q2 = q2/norm(q2);

p0=2*rand(1,7);p0(4:7) = q0;
p1=2*rand(1,7);p1(4:7) = q1;
p2=2*rand(1,7);p2(4:7) = q2;


clc;close all;
s=[0:0.01:1];
val = [];
tmax =2;
for i=1:length(s),
  val(:,i)=pose_foh([0:tmax],[p0' p1' p2'],s(i)*tmax);
end

figure(1)
hold on;
    plot3(val(1,:),val(2,:),val(3,:),'r.-','MarkerSize', 10); 
    view(3)
hold off;


figure(2)
hold on;

    plot3(val(4,:),val(5,:),val(6,:),'r.-','MarkerSize', 10); 
        draw_unitsphere(2);  
hold off;


val = [];
tmax =2;
for i=1:length(s),
    val(:,i)=pose_spline([0:tmax],[p0' p1' p2'],s(i)*tmax);
end
figure(1)
hold on;
    plot3(val(1,:),val(2,:),val(3,:),'g.-','MarkerSize', 10); 
    view(3)
    title('position spline')
hold off;


figure(2)
hold on;

    plot3(val(4,:),val(5,:),val(6,:),'g.-','MarkerSize', 10); 
        draw_unitsphere(2); 
        title('unit quaternion spline')
hold off;


figure(1)
