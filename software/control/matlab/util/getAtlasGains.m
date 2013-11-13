function gains = getAtlasGains(atlas_input_frame)
%GETATLASGAINS Returns default Atlas control gains

nq = 28; % fixed number

if atlas_input_frame.dim ~= nq
  error('joint_names has incorrect number of elements.');
end

gains = struct();
gains.ff_qd_d = zeros(nq,1);

k_f_p = Point(atlas_input_frame,0);
k_q_p = Point(atlas_input_frame,0);
k_q_i = Point(atlas_input_frame,0);
k_qd_p = Point(atlas_input_frame,0);
ff_qd = Point(atlas_input_frame,0);
ff_f_d = Point(atlas_input_frame,0);
ff_const = Point(atlas_input_frame,0);

% ff_const can always be zero now that we have a calibration routine 

% ff_const.neck_ay   =  0.0;
% ff_const.l_arm_usy =  0.1; % 9-19-13, fc
% ff_const.r_arm_usy = -0.075; % 10-09-13, fc
% ff_const.l_arm_shx = -0.01; % 9-17-13, fc
% ff_const.r_arm_shx =  0.055; % 9-17-13, fc
% ff_const.l_arm_ely = -0.045; % 9-17-13, fc
% ff_const.r_arm_ely =  -0.2; % 10-09-13, fc
% ff_const.l_arm_elx = -0.01; % 9-17-13, fc 
% ff_const.r_arm_elx = -0.025; % 9-18-13, fc
% ff_const.l_arm_uwy = -0.0875; % 9-17-13, fc
% ff_const.r_arm_uwy =  0.035; % 9-17-13, fc
% ff_const.l_arm_mwx =  0.025; % 9-17-13, fc
% ff_const.r_arm_mwx = -0.3; % 9-18-13, fc
% ff_const.l_leg_hpz =  0.27; % 10-03-13, fc 
% ff_const.l_leg_hpx = -0.0; % 10-07-13, fc 
% % ff_const.l_leg_hpx = -0.125; % 10-07-13, fc 
% % ff_const.l_leg_kny =  0.0; % 10-03-13, fc 
% ff_const.l_leg_kny =  0.05; % 10-09-13, fc 
% ff_const.l_leg_aky =  0.05; % 10-09-13, fc 
% ff_const.l_leg_akx =  0.135; % 10-03-13, fc 
% ff_const.r_leg_hpz =  0.0145; % 9-30-13, fc 
% ff_const.r_leg_hpy = -0.15; % 10-01-13, fc
% ff_const.r_leg_hpx =  0.05; % 10-03-13, fc 
% %ff_const.r_leg_hpx =  0.32; % 10-03-13, fc 
% ff_const.r_leg_kny =  0.025; % 10-08-13, fc 
% %ff_const.r_leg_kny =  0.055; % 10-03-13, fc 
% ff_const.r_leg_aky =  0.0; % 10-09-13, fc 
% ff_const.r_leg_akx =  -0.17; % 10-03-13, fc 

ff_qd.l_arm_usy = 0.3; % 9-19-13, fc
ff_qd.r_arm_usy = 0.3; % 9-19-13, fc
ff_qd.l_arm_shx = 0.275; % 9-19-13, fc 
ff_qd.r_arm_shx = 0.275; % 9-18-13, fc
ff_qd.l_arm_ely = 0.25; % 9-19-13, fc
ff_qd.r_arm_ely = 0.25; % 9-18-13, fc 
ff_qd.l_arm_elx = 0.3; % 9-19-13, fc
ff_qd.r_arm_elx = 0.25; % 9-18-13, fc
ff_qd.l_arm_uwy = 0.22; % 9-19-13, fc
ff_qd.r_arm_uwy = 0.22; % 9-18-13, fc
ff_qd.l_arm_mwx = 0.225; % 9-19-13, fc 
ff_qd.r_arm_mwx = 0.225; % 9-19-13, fc
ff_qd.l_leg_hpz = 0.4; 
ff_qd.l_leg_hpx = 1.35;  
ff_qd.l_leg_hpy = 1.0;  
ff_qd.l_leg_kny = 2.5; % 10-08-13, fc   
ff_qd.l_leg_aky = 1.275; % 10-15-13, fc 
ff_qd.l_leg_akx = 1.25; 
ff_qd.r_leg_hpz = 0.4; 
ff_qd.r_leg_hpy = 1.3; 
ff_qd.r_leg_hpx = 1.35;  
ff_qd.r_leg_kny = 1.4; % 10-15-13, fc   
ff_qd.r_leg_aky = 1.275; % 10-15-13, fc   
ff_qd.r_leg_akx = 1.25; % 10-03-13, fc  

% ff_f_d.l_arm_usy = 0.0; % 9-17-13
ff_f_d.r_arm_usy = 0.0025; % 9-17-13
% ff_f_d.l_arm_shx = 0.0; % 9-17-13
ff_f_d.r_arm_shx = 0.0; % 9-17-13
% ff_f_d.l_arm_elx = 0.0; % 9-17-13
ff_f_d.r_arm_elx = 0.005; % 9-17-13
% ff_f_d.l_arm_ely = 0.0; % 9-17-13
ff_f_d.r_arm_ely = 0.005; % 9-18-13
% ff_f_d.l_arm_uwy = 0.005; % 9-17-13
ff_f_d.r_arm_uwy = 0.0075; % 9-17-13
% ff_f_d.l_arm_mwx = 0.0; % 9-17-13
ff_f_d.r_arm_mwx = 0.01; % 9-17-13

k_f_p.back_bkz  = 0.012;
k_f_p.back_bky  = 0.012;
k_f_p.back_bkx  = 0.012;
k_f_p.neck_ay   = 0.012;
k_f_p.l_arm_usy = 0.08; % 9-17-13
k_f_p.l_arm_shx = 0.125; % 9-17-13
k_f_p.l_arm_ely = 0.115; % 9-17-13
k_f_p.l_arm_elx = 0.135; % 9-17-13
k_f_p.l_arm_uwy = 0.085; % 9-17-13
k_f_p.l_arm_mwx = 0.125; % 9-17-13
k_f_p.r_arm_usy = 0.09; % 10-09-13
k_f_p.r_arm_shx = 0.125; % 9-17-13
k_f_p.r_arm_ely = 0.125; % 9-17-13
k_f_p.r_arm_elx = 0.125; % 9-17-13
k_f_p.r_arm_uwy = 0.085; % 9-17-13
k_f_p.r_arm_mwx = 0.125; % 9-17-13
k_f_p.l_leg_hpz = 0.03; % 10-09-13
k_f_p.l_leg_hpx = 0.025; % 10-02-13
k_f_p.l_leg_hpy = 0.025; % 10-03-13 
k_f_p.l_leg_kny = 0.025; % 10-03-13
k_f_p.l_leg_aky = 0.4; % 10-03-13
k_f_p.l_leg_akx = 1.55; % 10-03-13
k_f_p.r_leg_hpz = 0.03; % 10-09-13
k_f_p.r_leg_hpy = 0.025; % 10-03-13
k_f_p.r_leg_hpx = 0.025; % 10-02-13
k_f_p.r_leg_kny = 0.02; % 10-03-13
k_f_p.r_leg_aky = 0.45; % 10-03-13
k_f_p.r_leg_akx = 1.7; % 10-03-13
  
% multiplying leg position gains by 5 so its safe to run while hanging

k_q_p.back_bkz  = 15.0;
k_q_p.back_bky  = 60.0;
k_q_p.back_bkx  = 60.0;
k_q_p.neck_ay   = 8.0;
k_q_p.l_leg_hpz = 45.0;% * 0.1;
k_q_p.l_leg_hpx = 45.0;% * 0.1;
k_q_p.l_leg_hpy = 55.0;% * 0.1;
k_q_p.l_leg_kny = 60.0;% * 0.1;
k_q_p.l_leg_aky = 1000.0;% * 0.1;
k_q_p.l_leg_akx = 1000.0;% * 0.1;
k_q_p.l_arm_usy = 10.5;
k_q_p.l_arm_shx = 7.0; % 11-13-13 seems sticky 
k_q_p.l_arm_ely = 7.0;
k_q_p.l_arm_elx = 12.0; % 11-13-13 seems sticky
k_q_p.l_arm_uwy = 10.0;
k_q_p.l_arm_mwx = 10.0;
k_q_p.r_leg_hpz = k_q_p.l_leg_hpz;
k_q_p.r_leg_hpx = k_q_p.l_leg_hpx;
k_q_p.r_leg_hpy = k_q_p.l_leg_hpy;
k_q_p.r_leg_kny = k_q_p.l_leg_kny;
k_q_p.r_leg_aky = k_q_p.l_leg_aky;
k_q_p.r_leg_akx = k_q_p.l_leg_akx;
k_q_p.r_arm_usy = 11.0;
k_q_p.r_arm_shx = 9.0; % 11-13-13 seems sticky 
k_q_p.r_arm_ely = 12.0; % 11-13-13
k_q_p.r_arm_elx = 12.0; % 11-13-13 seems sticky
k_q_p.r_arm_uwy = 15.0; % 11-13-13
k_q_p.r_arm_mwx = 14.0; % 11-13-13

k_qd_p.back_bkz  = 0.0;
k_qd_p.back_bky  = 0.0;
k_qd_p.back_bkx  = 0.0;
k_qd_p.neck_ay   = 0.1;
k_qd_p.l_leg_hpz = 0.0 * 0;
k_qd_p.l_leg_hpx = 0.0 * 0;
k_qd_p.l_leg_hpy = 0.1 * 0;
k_qd_p.l_leg_kny = 0.0 * 0;
k_qd_p.l_leg_aky = 2.5 * 0;
k_qd_p.l_leg_akx = 0.0 * 0;
k_qd_p.l_arm_usy = 0.5;
k_qd_p.l_arm_shx = 1.0; % 11-13-13 
k_qd_p.l_arm_ely = 0.5;
k_qd_p.l_arm_elx = 4.0; % 11-13-13 
k_qd_p.l_arm_uwy = 0.1;
k_qd_p.l_arm_mwx = 0.75;
k_qd_p.r_leg_hpz = k_qd_p.l_leg_hpz;
k_qd_p.r_leg_hpx = k_qd_p.l_leg_hpx;
k_qd_p.r_leg_hpy = k_qd_p.l_leg_hpy;
k_qd_p.r_leg_kny = k_qd_p.l_leg_kny;
k_qd_p.r_leg_aky = k_qd_p.l_leg_aky;
k_qd_p.r_leg_akx = k_qd_p.l_leg_akx;
k_qd_p.r_arm_usy = 0.5;
k_qd_p.r_arm_shx = 1.0; % 11-13-13
k_qd_p.r_arm_ely = 0.5; % 11-13-13
k_qd_p.r_arm_elx = 4.0; % 11-13-13
k_qd_p.r_arm_uwy = 0.5; % 11-13-13
k_qd_p.r_arm_mwx = 0.8; % 11-13-13 % slightly overdamped

gains.k_f_p = double(k_f_p);
gains.k_q_p = double(k_q_p);
gains.k_q_i = double(k_q_i);
gains.k_qd_p = double(k_qd_p);
gains.ff_f_d = double(ff_f_d);
gains.ff_const = double(ff_const);
gains.ff_qd = double(ff_qd);

end

