function v = rot2exp(R)

ang = acos(0.5*(trace(R)-1));
if (abs(ang)<1e-10)
    v = [0;0;0];
else
    cos_t = cos(ang);
    ax = sqrt((diag(R)-cos_t)/(1-cos_t));
    TODO
    v = ang*ax;
end
