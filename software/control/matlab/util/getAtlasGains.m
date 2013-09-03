function gains = getAtlasGains(atlas_input_frame)
%GETATLASGAINS Returns default Atlas control gains

nq = 28; % fixed number

if atlas_input_frame.dim ~= nq
  error('joint_names has incorrect number of elements.');
end

gains = struct();
gains.k_q_i = zeros(nq,1);
gains.ff_f_d = zeros(nq,1);
gains.ff_qd = zeros(nq,1);
gains.ff_qd_d = zeros(nq,1);

k_f_p = Point(atlas_input_frame,0);
k_q_p = Point(atlas_input_frame,0);
k_qd_p = Point(atlas_input_frame,0);
ff_f_d = Point(atlas_input_frame,0);
ff_const = Point(atlas_input_frame,0);

% set ff_const for subset of joints that need it
ff_const.l_arm_shx = 0.05; % 9-3-13, fc
ff_const.l_arm_elx = 0.1; % 9-3-13, fc 
ff_const.l_arm_uwy = -0.09; % 9-3-13, fc
ff_const.l_arm_mwx = -0.125; % 9-3-13, fc
ff_const.r_arm_usy = -0.05; % 9-3-13, fc
ff_const.r_arm_ely = -0.075; % 9-3-13, fc
ff_const.r_arm_uwy = -0.21; % 9-3-13, fc
ff_const.r_arm_mwx = -0.3; % 9-3-13, fc
ff_const.neck_ay = 0.1;

ff_f_d.l_arm_usy = 0.0025; % 9-3-13
ff_f_d.r_arm_usy = 0.0025; % 9-3-13
ff_f_d.l_arm_elx = 0.009; % 9-3-13
ff_f_d.r_arm_elx = 0.01; % 9-3-13
ff_f_d.l_arm_ely = 0.0; % 9-3-13
ff_f_d.r_arm_ely = 0.01; % 9-3-13
ff_f_d.l_arm_uwy = 0.0075; % 9-3-13
ff_f_d.r_arm_uwy = 0.005; % 9-3-13
ff_f_d.l_arm_mwx = 0.0; % 9-3-13
ff_f_d.r_arm_mwx = 0.01; % 9-3-13
 
k_f_p.back_bkz = 0.012;
k_f_p.back_bky = 0.012;
k_f_p.back_bkx = 0.012;
k_f_p.neck_ay = 0.012;
k_f_p.l_leg_hpz = 0.012;
k_f_p.l_leg_hpx = 0.012;
k_f_p.l_leg_hpy = 0.012;
k_f_p.l_leg_kny = 0.012;
k_f_p.l_leg_aky = 1.4;
k_f_p.l_leg_akx = 1.4;
k_f_p.l_arm_usy = 0.075; % 9-3-13
k_f_p.l_arm_shx = 0.125; % 9-3-13
k_f_p.l_arm_ely = 0.075; % 9-3-13
k_f_p.l_arm_elx = 0.12; % 9-3-13
k_f_p.l_arm_uwy = 0.075; % 9-3-13
k_f_p.l_arm_mwx = 0.1; % 9-3-13
k_f_p.r_leg_hpz = k_f_p.l_leg_hpz;
k_f_p.r_leg_hpx = k_f_p.l_leg_hpx;
k_f_p.r_leg_hpy = k_f_p.l_leg_hpy;
k_f_p.r_leg_kny = k_f_p.l_leg_kny;
k_f_p.r_leg_aky = k_f_p.l_leg_aky;
k_f_p.r_leg_akx = k_f_p.l_leg_akx;
k_f_p.r_arm_usy = 0.085; % 9-3-13
k_f_p.r_arm_shx = 0.125; % 9-3-13
k_f_p.r_arm_ely = 0.09; % 9-3-13
k_f_p.r_arm_elx = 0.085; % 9-3-13
k_f_p.r_arm_uwy = 0.085; % 9-3-13
k_f_p.r_arm_mwx = 0.1; % 9-3-13
  
k_q_p.back_bkz = 15.0;
k_q_p.back_bky = 60.0;
k_q_p.back_bkx = 60.0;
k_q_p.neck_ay = 8.0;
k_q_p.l_leg_hpz = 45.0;
k_q_p.l_leg_hpx = 30.0;
k_q_p.l_leg_hpy = 60.0;
k_q_p.l_leg_kny = 5.0;
k_q_p.l_leg_aky = 2000.0;
k_q_p.l_leg_akx = 2000.0;
k_q_p.l_arm_usy = 10.0;
k_q_p.l_arm_shx = 7.0;
k_q_p.l_arm_ely = 7.0;
k_q_p.l_arm_elx = 15.0;
k_q_p.l_arm_uwy = 10.0;
k_q_p.l_arm_mwx = 10.0;
k_q_p.r_leg_hpz = k_q_p.l_leg_hpz;
k_q_p.r_leg_hpx = k_q_p.l_leg_hpx;
k_q_p.r_leg_hpy = k_q_p.l_leg_hpy;
k_q_p.r_leg_kny = k_q_p.l_leg_kny;
k_q_p.r_leg_aky = k_q_p.l_leg_aky;
k_q_p.r_leg_akx = k_q_p.l_leg_akx;
k_q_p.r_arm_usy = 10.0;
k_q_p.r_arm_shx = 7.0; 
k_q_p.r_arm_ely = 8.0;
k_q_p.r_arm_elx = 15.0;
k_q_p.r_arm_uwy = 17.0;
k_q_p.r_arm_mwx = 13.0;

k_qd_p.back_bkz = 0.0;
k_qd_p.back_bky = 0.0;
k_qd_p.back_bkx = 0.0;
k_qd_p.neck_ay = 0.1;
k_qd_p.l_leg_hpz = 0.0;
k_qd_p.l_leg_hpx = 0.0;
k_qd_p.l_leg_hpy = 0.0;
k_qd_p.l_leg_kny = 0.0;
k_qd_p.l_leg_aky = 0.0;
k_qd_p.l_leg_akx = 0.0;
k_qd_p.l_arm_usy = 0.5;
k_qd_p.l_arm_shx = 0.9; 
k_qd_p.l_arm_ely = 0.5;
k_qd_p.l_arm_elx = 0.8;
k_qd_p.l_arm_uwy = 0.1;
k_qd_p.l_arm_mwx = 0.75;
k_qd_p.r_leg_hpz = k_qd_p.l_leg_hpz;
k_qd_p.r_leg_hpx = k_qd_p.l_leg_hpx;
k_qd_p.r_leg_hpy = k_qd_p.l_leg_hpy;
k_qd_p.r_leg_kny = k_qd_p.l_leg_kny;
k_qd_p.r_leg_aky = k_qd_p.l_leg_aky;
k_qd_p.r_leg_akx = k_qd_p.l_leg_akx;
k_qd_p.r_arm_usy = 0.5;
k_qd_p.r_arm_shx = 0.9; 
k_qd_p.r_arm_ely = 0.5;
k_qd_p.r_arm_elx = 0.65;
k_qd_p.r_arm_uwy = 0.1;
k_qd_p.r_arm_mwx = 0.25; %note: seems to be sensitive--velocity noise

gains.k_f_p = double(k_f_p);
gains.k_q_p = double(k_q_p);
gains.k_qd_p = double(k_qd_p);
gains.ff_f_d = double(ff_f_d);
gains.ff_const = double(ff_const);

end

