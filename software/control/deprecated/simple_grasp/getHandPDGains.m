function [Kp,Kd] = getHandPDGains(manip)

%NOTEST

B = manip.getB; %For use in trunk
% B=manip.manip.B; %For use in scott's branch
idx = B'*(1:manip.getNumStates()/2)';

fr=CoordinateFrame('q_d',length(idx),'d',{manip.getStateFrame.coordinates{idx}});
Kp = Point(fr);
Kd = Point(fr);

Kp.right_f0_j0 = 12.0;
Kp.right_f0_j1 = 8.0;
% Kp.right_f0_j2 = 20.0;
Kp.right_f0_j2 = 10.0;
Kp.right_f1_j0 = 12.0;
Kp.right_f1_j1 = 8.0;
Kp.right_f1_j2 = 8.0;
Kp.right_f2_j0 = 12.0;
Kp.right_f2_j1 = 8.0;
Kp.right_f2_j2 = 8.0;
Kp.right_f3_j0 = 12.0;
Kp.right_f3_j1 = 8.0;
Kp.right_f3_j2 = 8.0;

Kd.right_f0_j0 = 1.4;
Kd.right_f0_j1 = 0.85;
% Kd.right_f0_j2 = 1.8;
Kd.right_f0_j2 = 0.9;
Kd.right_f1_j0 = 1.4;
Kd.right_f1_j1 = 0.9;
Kd.right_f1_j2 = 1.1;
Kd.right_f2_j0 = 1.4;
Kd.right_f2_j1 = 0.9;
Kd.right_f2_j2 = 1.1;
Kd.right_f3_j0 = 1.4;
Kd.right_f3_j1 = 0.9;
Kd.right_f3_j2 = 1.1;

Kp = diag(double(Kp));
Kd = diag(double(Kd));

end