function R = rpy2rot(rpy)

cos_a = cos(rpy(3));
sin_a = sin(rpy(3));
cos_b = cos(rpy(2));
sin_b = sin(rpy(2));
cos_c = cos(rpy(1));
sin_c = sin(rpy(1));

R = [cos_a*cos_b, cos_a*sin_b*sin_c - sin_a*cos_c, cos_a*sin_b*cos_c + sin_a*sin_c;
    sin_a*cos_b, sin_a*sin_b*sin_c + cos_a*cos_c, sin_a*sin_b*cos_c - cos_a*sin_c;
    -sin_b, cos_b*sin_c, cos_b*cos_c];

