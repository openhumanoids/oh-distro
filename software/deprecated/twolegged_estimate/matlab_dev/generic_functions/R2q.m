function [q] = R2q(rot) 
% Convert rotation matrix to a quaternion, with fixed reference frame
% from MIT libbot

    quat(1) = 0.5*sqrt(rot(1,1)+rot(2,2)+rot(3,3)+1);

    if (abs(quat(1)) > 1e-8) 
      w4 = 1.0/(4.0*quat(1));
      quat(2) = (rot(3,2)-rot(2,3)) * w4;
      quat(3) = (rot(1,3)-rot(3,1)) * w4;
      quat(4) = (rot(2,1)-rot(1,2)) * w4;
    
    else 
      quat(2) = sqrt(abs(-0.5*(rot(2,2)+rot(3,3))));
      quat(3) = sqrt(abs(-0.5*(rot(1,1)+rot(3,3))));
      quat(4) = sqrt(abs(-0.5*(rot(1,1)+rot(2,2))));
    end

    %/*LSF: I may be missing something but this didn't work for me until I divided by the magnitude instead:
    %  double norm = quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] +
    %                   quat[3]*quat[3];
    %if (fabs(norm) < 1e-10)
    %return -1; */
    quat = quat/norm(quat);
    q = quat;

