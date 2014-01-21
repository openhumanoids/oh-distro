function inertialData = init_inertialData(g)


inertialData.gw = [0;0;g];
inertialData.predicted.utime = 0;
inertialData.measured.w_b = -999999*[1;1;1];
inertialData.measured.a_b = +999999*[1;1;1];
inertialData.predicted.w_b = inertialData.measured.w_b;
inertialData.predicted.a_b = inertialData.measured.a_b;


