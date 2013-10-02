close all;
clear all;
roll =0;   pitch = 0;  yaw = 0.0;
q0 = rpy2quat(  [roll, pitch, yaw])';
roll =pi;   pitch =-pi/2;  yaw = 0.0;
q1 = rpy2quat(  [roll, pitch, yaw])';
roll =0;   pitch = pi/2;  yaw = 0.0;
q2 = rpy2quat(  [roll, pitch, yaw])';
roll =0;   pitch =-2*pi;  yaw = 0.0;
q3 = rpy2quat(  [roll, pitch, yaw])';

%q0 = rand(1,4);q1=rand(1,4);
% by setting one dimension to zero, we can visualize the quaternion
% interpolation on a 3d Sphere (S2)
q0(4)=0;q1(4)=0;q2(4)=0;
q0 = q0/norm(q0);
q1 = q1/norm(q1);
q2 = q2/norm(q2);

%q3=rand(1,4);
q4=rand(1,4);

q3(4)=0;q4(4)=0;
q3 = q3/norm(q3);
q4 = q4/norm(q4);


close all;
figure(1);
s=[0:0.01:1];
val = [];angvel_foh=[];
for i=1:length(s),
    val(:,i)=quat_foh([q0' q1' q2' q3'],s(i));
    %val=val(1:3)/norm(val(1:3));
    if(i>2)
    omega=quatlog(quatmultiply(quatinv(val(:,i-1)'),val(:,i)'));
    angvel_foh = [angvel_foh quatexp(omega)'];
    end

end;
hold on;
    plot3(val(1,:),val(2,:),val(3,:),'r.-','MarkerSize', 10); 
hold off;

val = [];
angvel_squad=[];
for i=1:length(s),
    val(:,i)=quat_spline([q0' q1' q2' q3'],s(i),'squad');
    %val=val(1:3)/norm(val(1:3));
    if(i>2)
    omega=quatlog(quatmultiply(quatinv(val(:,i-1)'),val(:,i)'));
    angvel_squad = [angvel_squad quatexp(omega)'];
    end

end;
hold on;
    plot3(val(1,:),val(2,:),val(3,:),'b.-','MarkerSize', 10); 
hold off;


val = [];
angvel_spline=[];
disp(' ')
for i=1:length(s),
    %Bezier curves
    %val=bezier_quat_curvefit([q0' q1' q2'],s(i)); % Does not pass through
    %all the points.
    %val(:,i)=hermite_quat_splinefit([q0' q1' q2' q3' q0'],s(i));
    val(:,i)=quat_spline([q0' q1' q2' q3'],s(i),'hermite_cubic');
    val(1:3,i)=val(1:3,i)/norm(val(1:3,i));
    if(i>2)
    omega=quatlog(quatmultiply(quatinv(val(:,i-1)'),val(:,i)'));
    angvel_spline = [angvel_spline quatexp(omega)'];
    end
end; 
hold on;
    plot3(val(1,:),val(2,:),val(3,:),'g.-','MarkerSize', 10); 
draw_unitsphere(gcf);   
legend('pw-slerp','squad-quadrangle interp','spherical-bezier-cubic-spline')

view(3);
hold off;

figure;
hold on;
plot(angvel_foh(1,:),'b.-','MarkerSize', 10); 
plot(angvel_foh(2,:),'g.-','MarkerSize', 10); 
plot(angvel_foh(3,:),'r.-','MarkerSize', 10); 
plot(angvel_foh(4,:),'m.-','MarkerSize', 10); 
title('angular velocity of foh')
hold off;

figure;
hold on;
plot(angvel_squad(1,:),'b.-','MarkerSize', 10); 
plot(angvel_squad(2,:),'g.-','MarkerSize', 10); 
plot(angvel_squad(3,:),'r.-','MarkerSize', 10); 
plot(angvel_squad(4,:),'m.-','MarkerSize', 10); 
title('angular velocity of squad')
hold off;


figure;
hold on;
plot(angvel_spline(1,:),'b.-','MarkerSize', 10); 
plot(angvel_spline(2,:),'g.-','MarkerSize', 10); 
plot(angvel_spline(3,:),'r.-','MarkerSize', 10); 
plot(angvel_spline(4,:),'m.-','MarkerSize', 10); 
title('angular velocity of bezier-cubic-spline')
hold off;

figure(1);
