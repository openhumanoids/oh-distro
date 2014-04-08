function out = slerp(in1,in2,t)
in1 = in1/norm(in1);
in2 = in2/norm(in2);
angle = abs(acos(dot(in1,in2)));
sin_angle = sin(angle);
if (sin_angle==0)
    out = in1;
else
    out = sin((1-t)*angle)/sin(angle)*in1 + sin(t*angle)/sin(angle)*in2;
end