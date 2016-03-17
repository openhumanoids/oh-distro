function q = rot2quat(R)

tr = trace(R)+1;
if (tr > 1e-8)
    s = 2*sqrt(tr);
    x = (R(3,2) - R(2,3)) / s;
    y = (R(1,3) - R(3,1)) / s;
    z = (R(2,1) - R(1,2)) / s;
    w = 0.25 * s;
else
    if (R(1,1)>R(2,2) && R(1,1)>R(3,3))
        s  = sqrt( 1.0 + R(1,1) - R(2,2) - R(3,3) ) * 2;
        x = 0.25 * s;
        y = (R(2,1) + R(1,2) ) / s;
        z = (R(1,3) + R(3,1) ) / s;
        w = (R(3,2) - R(2,3) ) / s;

    elseif ( R(2,2) > R(3,3) )
        s  = sqrt( 1.0 + R(2,2) - R(1,1) - R(3,3) ) * 2;
        x = (R(2,1) + R(1,2) ) / s;
        y = 0.25 * s;
        z = (R(3,2) + R(2,3) ) / s;
        w = (R(1,3) - R(3,1) ) / s;
    else
        s  = sqrt( 1.0 + R(3,3) - R(1,1) - R(2,2) ) * 2;
        x = (R(1,3) + R(3,1) ) / s;
        y = (R(3,2) + R(2,3) ) / s;
        z = 0.25 * s;
        w = (R(2,1) - R(1,2) ) / s;
    end
end
q = [w;x;y;z];
        % 0 1  2
        % 4 5  6
        % 8 9 10
q = q/norm(q);



% w = 0.5*sqrt(trace(R)+1);
% 
% if (abs(w) > 1e-8)
%     scale = 1/(4*w);
%     x = (R(3,2)-R(2,3))*scale;
%     y = (R(1,3)-R(3,1))*scale;
%     z = (R(2,1)-R(1,2))*scale;
% else
%     x = sqrt(abs(-0.5*(R(2,2)+R(3,3))));
%     y = sqrt(abs(-0.5*(R(1,1)+R(3,3))));
%     z = sqrt(abs(-0.5*(R(1,1)+R(2,2))));
% end
% 
% q = [x;y;z;w];
% q = q/norm(q);
