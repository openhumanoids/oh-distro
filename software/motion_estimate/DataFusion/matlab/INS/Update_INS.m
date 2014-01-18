function [pose, INSCompensator] = Update_INS(pose__, INSCompensator)


% time first
if (inertialData.predicted.utime<pose__.utime)
    disp('Update_INS.m: ERROR, you cannot work backwards in time.');
end

% predict local frame accelerations
pose.al = qrot(qconj(pose__k1.lQb),inertialData.predicted.ab);
pose.fl = (inertialData.predicted.al - inertialData.gw)';
pose.P_l = pose__k1.pl + 0.5*dt*(pose__k1.vl + pose__k2.vl);
pose.V_l = pose__k1.vl + 0.5*dt*(pose__k1.fl + pose__k2.fl);

pose.lQb = zeroth_int_Quat_closed_form(-inertialData.predicted.wb, pose__k1.lQb, dt);




