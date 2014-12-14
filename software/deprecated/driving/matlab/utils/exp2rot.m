function R = exp2rot(v)

t = norm(v);
if (t<1e-10)
    R = eye(3);
    return;
end

v = v/t;
vcross = [
    0, -v(3), v(2);
    v(3), 0, -v(1);
    -v(2), v(1), 0
    ];

R = eye(3) + vcross*sin(t) + vcross*vcross*(1-cos(t));
