function gains = getAtlasGains()
%GETATLASGAINS Returns default Atlas control gains

% fixed input frame ordering
back_bkz  = 1;
back_bky  = 2;
back_bkx  = 3;
l_arm_elx = 4;
l_arm_ely = 5;
l_arm_mwx = 6;
l_arm_shx = 7;
l_arm_usy = 8;
l_arm_uwy = 9;
l_leg_kny = 10;
l_leg_akx = 11;
l_leg_hpy = 12;
l_leg_hpx = 13;
l_leg_aky = 14;
l_leg_hpz = 15;
neck_ay   = 16;
r_arm_elx = 17;
r_arm_ely = 18;
r_arm_mwx = 19;
r_arm_shx = 20;
r_arm_usy = 21;
r_arm_uwy = 22;
r_leg_kny = 23;
r_leg_akx = 24;
r_leg_hpy = 25;
r_leg_hpx = 26;
r_leg_aky = 27;
r_leg_hpz = 28;

nu = 28;

k_f_p    = zeros(nu,1);
k_q_p    = zeros(nu,1);
k_q_i    = zeros(nu,1);
k_qd_p   = zeros(nu,1);
ff_qd    = zeros(nu,1);
ff_f_d   = zeros(nu,1);
ff_const = zeros(nu,1);
ff_qd_d  = zeros(nu,1);

% position, proportunal
k_q_p(back_bkz)  = 20.0;
k_q_p(back_bky)  = 60.0;
k_q_p(back_bkx)  = 60.0;
k_q_p(neck_ay)   = 8.0;
k_q_p(l_leg_hpz) = 45.0;
k_q_p(l_leg_hpx) = 30.0;
k_q_p(l_leg_hpy) = 50.0;
k_q_p(l_leg_kny) = 30.0;
k_q_p(l_leg_aky) = 1000.0;
k_q_p(l_leg_akx) = 1000.0;
k_q_p(l_arm_usy) = 4.0; 
k_q_p(l_arm_shx) = 4.0;  
k_q_p(l_arm_ely) = 4.0; 
k_q_p(l_arm_elx) = 4.0; 
k_q_p(l_arm_uwy) = 4.0; 
k_q_p(l_arm_mwx) = 4.0; 
k_q_p(r_leg_hpz) = k_q_p(l_leg_hpz);
k_q_p(r_leg_hpx) = k_q_p(l_leg_hpx);
k_q_p(r_leg_hpy) = k_q_p(l_leg_hpy);
k_q_p(r_leg_kny) = k_q_p(l_leg_kny);
k_q_p(r_leg_aky) = k_q_p(l_leg_aky);
k_q_p(r_leg_akx) = k_q_p(l_leg_akx);
k_q_p(r_arm_usy) = 4.0; 
k_q_p(r_arm_shx) = 4.0;  
k_q_p(r_arm_ely) = 4.0; 
k_q_p(r_arm_elx) = 4.0; 
k_q_p(r_arm_uwy) = 4.0; 
k_q_p(r_arm_mwx) = 4.0; 


% velocity, proportunal
k_qd_p(back_bkz)  = 0.5;
k_qd_p(back_bky)  = 1.0;
k_qd_p(back_bkx)  = 1.0;
k_qd_p(neck_ay)   = 0.1;


% force, proportunal
k_f_p(back_bkz)  = 0.005;
k_f_p(back_bky)  = 0.02;
k_f_p(back_bkx)  = 0.02;
k_f_p(l_leg_hpz) = 0.02; % 02-03-14, f+v 
k_f_p(l_leg_hpx) = 0.03; % 02-03-14, f+v 
k_f_p(l_leg_hpy) = 0.02; % 02-03-14, f+v
k_f_p(l_leg_kny) = 0.02; % 02-03-14, f+v
k_f_p(l_leg_aky) = 0.6; % 02-03-14, f+v
k_f_p(l_leg_akx) = 0.75; % 02-03-14, f+v 
k_f_p(r_leg_hpz) = 0.02; % 02-03-14, f+v 
k_f_p(r_leg_hpx) = 0.03; % 02-03-14, f+v 
k_f_p(r_leg_hpy) = 0.02; % 02-03-14, f+v 
k_f_p(r_leg_kny) = 0.02; % 02-03-14, f+v 
k_f_p(r_leg_aky) = 0.6; % 02-03-14, f+v
k_f_p(r_leg_akx) = 0.75; % 02-03-14, f+v 


% velocity, feedforward
ff_qd_d(back_bkz)  = 1.0;
ff_qd_d(back_bky)  = 3.0;
ff_qd_d(back_bkx)  = 3.0;
ff_qd_d(l_leg_hpz) = 2.0; % 03-24-14, f+v 
ff_qd_d(l_leg_hpx) = 4.0; % 03-24-14, f+v 
ff_qd_d(l_leg_hpy) = 4.0; % 03-24-14, f+v
ff_qd_d(l_leg_kny) = 4.0; % 03-24-14, f+v
ff_qd_d(l_leg_aky) = 40.0; 
ff_qd_d(l_leg_akx) = 40.0; 
ff_qd_d(r_leg_hpz) = 2.0; % 03-24-14, f+v 
ff_qd_d(r_leg_hpx) = 4.0; % 03-24-14, f+v  
ff_qd_d(r_leg_hpy) = 4.0; % 03-24-14, f+v 
ff_qd_d(r_leg_kny) = 4.0; % 03-24-14, f+v
ff_qd_d(r_leg_aky) = 40.0; 
ff_qd_d(r_leg_akx) = 40.0; 


gains = struct();
gains.k_f_p = k_f_p;
gains.k_q_p = k_q_p;
gains.k_q_i = k_q_i;
gains.k_qd_p = k_qd_p;
gains.ff_f_d = ff_f_d;
gains.ff_const = ff_const;
gains.ff_qd = ff_qd;
gains.ff_qd_d = ff_qd_d;

end