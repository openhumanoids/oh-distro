function REQM = initDFReqMsgData(iter, REQM)


% fix ordering for user readability
REQM.utime = zeros(iter);
REQM.P_l = zeros(iter,3);
REQM.V_l = zeros(iter,3);
REQM.lQb = zeros(iter,3);
REQM.a_l = zeros(iter,3);
REQM.f_l = zeros(iter,3);

REQM.a_b = zeros(iter,3);
REQM.w_b = zeros(iter,3);
REQM.ba = zeros(iter,3);
REQM.bg = zeros(iter,3);
