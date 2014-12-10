function R = q2R(q)
% EXPERIMENTAL
% Convert quaternion to linear rotation matrix.
% Fixed frame rotation
% q = [scalar vector]

    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    nrm = norm(q);
    if (abs(nrm) < 0.9)
        disp('QuaternionLib::q2C -- not a unit quaternion');
        R = eye(3);
        return;
    end

		nrm = 1./nrm;
		w = w*nrm;
		x = x*nrm;
		y = y*nrm;
		z = z*nrm;

        x2 = x*x;
        y2 = y*y;
        z2 = z*z;
        w2 = w*w;
        xy = 2*x*y;
        xz = 2*x*z;
        yz = 2*y*z;
        wx = 2*w*x;
        wy = 2*w*y;
        wz = 2*w*z;

        R = zeros(3);
        
		R(1,1) = w2+x2-y2-z2;  R(1,2) = xy-wz;  R(1,3) = xz+wy;
		R(2,1) = xy+wz;  R(2,2) = w2-x2+y2-z2;  R(2,3) = yz-wx;
		R(3,1) = xz-wy;  R(3,2) = yz+wx;  R(3,3) = w2-x2-y2+z2;

end