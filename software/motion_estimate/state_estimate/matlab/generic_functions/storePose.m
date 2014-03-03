function [ Poses ] = storePose( pose, Poses, k )


Poses.lQb(k,:) = pose.lQb';
Poses.a_l(k,:) = pose.a_l';
Poses.f_l(k,:) = pose.f_l';
Poses.V_l(k,:) = pose.V_l';
Poses.P_l(k,:) = pose.P_l';

end

