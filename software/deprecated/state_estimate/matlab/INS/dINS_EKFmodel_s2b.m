function [F,L,Q] = dINS_EKFmodel_s2b(pose, sRb)
    
% disp(['dINS_EKFmodel_s2b -- ' num2str(pose.lQb')]);
    
F = zeros(15);
F(1:3,4:6) = -q2R(qconj(pose.lQb)) * sRb;
F(7:9,1:3) = -vec2skew(pose.a_l);
F(7:9,10:12) = -q2R(qconj(pose.lQb)) * sRb;
F(13:15,7:9) = eye(3);

Q = 1*diag([0*ones(1,3), 1E-5*ones(1,3), 0*ones(1,3), 1E-4*ones(1,3), 0*ones(1,3)]);

% uneasy about the negative signs, but this is what we have in
% literature (noise is noise, right.)
L = blkdiag(eye(3), -eye(3), -q2R(qconj(pose.lQb))*sRb, eye(3), eye(3));


return

