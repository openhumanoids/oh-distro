function gains = getAtlasGains(atlas_input_frame)
%GETATLASGAINS Returns default Atlas control gains

nq = 28; % fixed number

if atlas_input_frame.dim ~= nq
  error('joint_names has incorrect number of elements.');
end

gains = struct();

k_f_p = Point(atlas_input_frame,0);
k_q_p = Point(atlas_input_frame,0);
k_q_i = Point(atlas_input_frame,0);
k_qd_p = Point(atlas_input_frame,0);
ff_qd = Point(atlas_input_frame,0);
ff_f_d = Point(atlas_input_frame,0);
ff_const = Point(atlas_input_frame,0);
ff_qd_d = Point(atlas_input_frame,0);

% ff_const can always be zero now that we have a calibration routine
% ff_const.neck_ay = 0.0;
% ff_const.l_arm_usy = 0.1; % 9-19-13, fc
% ff_const.r_arm_usy = -0.075; % 10-09-13, fc
% ff_const.l_arm_shx = -0.01; % 9-17-13, fc
% ff_const.r_arm_shx = 0.055; % 9-17-13, fc
% ff_const.l_arm_ely = -0.045; % 9-17-13, fc
% ff_const.r_arm_ely = -0.2; % 10-09-13, fc
% ff_const.l_arm_elx = -0.01; % 9-17-13, fc
% ff_const.r_arm_elx = -0.025; % 9-18-13, fc
% ff_const.l_arm_uwy = -0.0875; % 9-17-13, fc
% ff_const.r_arm_uwy = 0.035; % 9-17-13, fc
% ff_const.l_arm_mwx = 0.025; % 9-17-13, fc
% ff_const.r_arm_mwx = -0.3; % 9-18-13, fc
% ff_const.l_leg_hpz = 0.27; % 10-03-13, fc
% ff_const.l_leg_hpx = -0.0; % 10-07-13, fc
% % ff_const.l_leg_hpx = -0.125; % 10-07-13, fc
% % ff_const.l_leg_kny = 0.0; % 10-03-13, fc
% ff_const.l_leg_kny = 0.05; % 10-09-13, fc
% ff_const.l_leg_aky = 0.05; % 10-09-13, fc
% ff_const.l_leg_akx = 0.135; % 10-03-13, fc
% ff_const.r_leg_hpz = 0.0145; % 9-30-13, fc
% ff_const.r_leg_hpy = -0.15; % 10-01-13, fc
% ff_const.r_leg_hpx = 0.05; % 10-03-13, fc
% %ff_const.r_leg_hpx = 0.32; % 10-03-13, fc
% ff_const.r_leg_kny = 0.025; % 10-08-13, fc
% %ff_const.r_leg_kny = 0.055; % 10-03-13, fc
% ff_const.r_leg_aky = 0.0; % 10-09-13, fc
% ff_const.r_leg_akx = -0.17; % 10-03-13, fc

k_q_p.back_bkz  = 20.0;
k_q_p.back_bky  = 60.0;
k_q_p.back_bkx  = 60.0;
k_q_p.neck_ay   = 8.0;
k_q_p.l_leg_hpz = 45.0;
k_q_p.l_leg_hpx = 30.0;
k_q_p.l_leg_hpy = 50.0;
k_q_p.l_leg_kny = 30.0;
k_q_p.l_leg_aky = 1000.0;
k_q_p.l_leg_akx = 1000.0;
k_q_p.l_arm_usy = 4.0; 
k_q_p.l_arm_shx = 4.0;  
k_q_p.l_arm_ely = 4.0; 
k_q_p.l_arm_elx = 4.0; 
k_q_p.l_arm_uwy = 4.0; 
k_q_p.l_arm_mwx = 4.0; 
k_q_p.r_leg_hpz = k_q_p.l_leg_hpz;
k_q_p.r_leg_hpx = k_q_p.l_leg_hpx;
k_q_p.r_leg_hpy = k_q_p.l_leg_hpy;
k_q_p.r_leg_kny = k_q_p.l_leg_kny;
k_q_p.r_leg_aky = k_q_p.l_leg_aky;
k_q_p.r_leg_akx = k_q_p.l_leg_akx;
k_q_p.r_arm_usy = 4.0; 
k_q_p.r_arm_shx = 4.0;  
k_q_p.r_arm_ely = 4.0; 
k_q_p.r_arm_elx = 4.0; 
k_q_p.r_arm_uwy = 4.0; 
k_q_p.r_arm_mwx = 4.0; 

k_qd_p.back_bkz  = 0.5;
k_qd_p.back_bky  = 1.0;
k_qd_p.back_bkx  = 1.0;
k_qd_p.neck_ay   = 0.1;
% k_qd_p.l_leg_hpz = 0.1;
% k_qd_p.l_leg_hpx = 0.1;
% k_qd_p.l_leg_hpy = 0.2;
% k_qd_p.l_leg_kny = 0.2;
% k_qd_p.l_leg_aky = 2.5;
% k_qd_p.l_leg_akx = 0.1;
% k_qd_p.l_arm_usy = 0.85; % 11-14-13
% k_qd_p.l_arm_shx = 2.7; % 11-14-13 
% k_qd_p.l_arm_ely = 2.25; % 11-14-13
% k_qd_p.l_arm_elx = 10.5; % 11-14-13--yikes
% k_qd_p.l_arm_uwy = 0.5; % 11-14-13
% k_qd_p.l_arm_mwx = 0.95;% 11-14-13
% k_qd_p.r_leg_hpz = k_qd_p.l_leg_hpz;
% k_qd_p.r_leg_hpx = k_qd_p.l_leg_hpx;
% k_qd_p.r_leg_hpy = k_qd_p.l_leg_hpy;
% k_qd_p.r_leg_kny = k_qd_p.l_leg_kny;
% k_qd_p.r_leg_aky = k_qd_p.l_leg_aky;
% k_qd_p.r_leg_akx = k_qd_p.l_leg_akx;
% k_qd_p.r_arm_usy = 1.5; % 11-14-13
% k_qd_p.r_arm_shx = 1.6; % 11-14-13
% k_qd_p.r_arm_ely = 0.85; % 11-14-13
% k_qd_p.r_arm_elx = 10.5; % 11-13-13--yikes
% k_qd_p.r_arm_uwy = 0.5; % 11-13-13
% k_qd_p.r_arm_mwx = 1.0; % 11-14-13










% ff_f_d.back_bkz  = 0.0;
% ff_f_d.back_bky  = 0.0;
% ff_f_d.back_bkx  = 0.0;
% ff_f_d.neck_ay   = 0.0;
% ff_f_d.l_arm_usy = 0.0; 
% ff_f_d.l_arm_shx = 0.0;  
% ff_f_d.l_arm_ely = 0.0; 
% ff_f_d.l_arm_elx = 0.0; 
% ff_f_d.l_arm_uwy = 0.0; 
% ff_f_d.l_arm_mwx = 0.0;
% ff_f_d.r_arm_usy = 0.0; 
% ff_f_d.r_arm_shx = 0.0; 
% ff_f_d.r_arm_ely = 0.0; 
% ff_f_d.r_arm_elx = 0.0; 
% ff_f_d.r_arm_uwy = 0.0; 
% ff_f_d.r_arm_mwx = 0.0; 
ff_f_d.l_leg_hpz = 0.0;
ff_f_d.l_leg_hpx = 0.0;
ff_f_d.l_leg_hpy = 0.0; 
ff_f_d.l_leg_kny = 0.0; 
ff_f_d.l_leg_aky = 0.0;
ff_f_d.l_leg_akx = 0.0;
ff_f_d.r_leg_hpz = 0.0;
ff_f_d.r_leg_hpx = 0.0;
ff_f_d.r_leg_hpy = 0.0;  
ff_f_d.r_leg_kny = 0.0;
ff_f_d.r_leg_aky = 0.0;
ff_f_d.r_leg_akx = 0.0;


% k_f_p.back_bkz  = 0.012;
% k_f_p.back_bky  = 0.012;
% k_f_p.back_bkx  = 0.012;
% k_f_p.l_arm_usy = 0.08; % 9-17-13
% k_f_p.l_arm_shx = 0.125; % 9-17-13
% k_f_p.l_arm_ely = 0.115; % 9-17-13
% k_f_p.l_arm_elx = 0.135; % 9-17-13
% k_f_p.l_arm_uwy = 0.085; % 9-17-13
% k_f_p.l_arm_mwx = 0.125; % 9-17-13
% k_f_p.r_arm_usy = 0.09; % 10-09-13
% k_f_p.r_arm_shx = 0.125; % 9-17-13
% k_f_p.r_arm_ely = 0.125; % 9-17-13
% k_f_p.r_arm_elx = 0.125; % 9-17-13
% k_f_p.r_arm_uwy = 0.085; % 9-17-13
% k_f_p.r_arm_mwx = 0.125; % 9-17-13
k_f_p.l_leg_hpz = 0.02; % 02-03-14, f+v 
k_f_p.l_leg_hpx = 0.02; % 02-03-14, f+v 
k_f_p.l_leg_hpy = 0.02; % 02-03-14, f+v
k_f_p.l_leg_kny = 0.0175; % 02-03-14, f+v
k_f_p.l_leg_aky = 0.3; % 02-03-14, f+v
k_f_p.l_leg_akx = 0.5; % 02-03-14, f+v 
k_f_p.r_leg_hpz = 0.02; % 02-03-14, f+v 
k_f_p.r_leg_hpx = 0.02; % 02-03-14, f+v 
k_f_p.r_leg_hpy = 0.02; % 02-03-14, f+v 
k_f_p.r_leg_kny = 0.0175; % 02-03-14, f+v 
k_f_p.r_leg_aky = 0.3; % 02-03-14, f+v
k_f_p.r_leg_akx = 0.5; % 02-03-14, f+v 


% ff_qd.l_arm_usy = 0.3; % 9-19-13, fc
% ff_qd.r_arm_usy = 0.3; % 9-19-13, fc
% ff_qd.l_arm_shx = 0.275; % 9-19-13, fc 
% ff_qd.r_arm_shx = 0.275; % 9-18-13, fc
% ff_qd.l_arm_ely = 0.25; % 9-19-13, fc
% ff_qd.r_arm_ely = 0.25; % 9-18-13, fc 
% ff_qd.l_arm_elx = 0.3; % 9-19-13, fc
% ff_qd.r_arm_elx = 0.25; % 9-18-13, fc
% ff_qd.l_arm_uwy = 0.22; % 9-19-13, fc
% ff_qd.r_arm_uwy = 0.22; % 9-18-13, fc
% ff_qd.l_arm_mwx = 0.225; % 9-19-13, fc 
% ff_qd.r_arm_mwx = 0.225; % 9-19-13, fc
ff_qd.l_leg_hpz = 0.1; % 02-03-14, f+v 
ff_qd.l_leg_hpx = 0.1; % 02-03-14, f+v   
ff_qd.l_leg_hpy = 0.09; % 02-03-14, f+v 
ff_qd.l_leg_kny = 0.03; % 02-03-14, f+v     
ff_qd.l_leg_aky = 0.5; % 02-03-14, f+v  
ff_qd.l_leg_akx = 0.65; % 02-04-14, f+v
ff_qd.r_leg_hpz = 0.1; % 02-03-14, f+v
ff_qd.r_leg_hpx = 0.1; % 02-03-14, f+v 
ff_qd.r_leg_hpy = 0.09; % 02-03-14, f+v 
ff_qd.r_leg_kny = 0.03; % 02-03-14, f+v 
ff_qd.r_leg_aky = 0.5; % 02-03-14, f+v    
ff_qd.r_leg_akx = 0.65; % 02-04-14, f+v   

% ff_qd_d.back_bkz  = 0.0;
% ff_qd_d.back_bky  = 0.0;
% ff_qd_d.back_bkx  = 0.0;
% ff_qd_d.l_arm_usy = 0.0; 
% ff_qd_d.l_arm_shx = 0.0;  
% ff_qd_d.l_arm_ely = 0.0; 
% ff_qd_d.l_arm_elx = 0.0; 
% ff_qd_d.l_arm_uwy = 0.0; 
% ff_qd_d.l_arm_mwx = 0.0;
% ff_qd_d.r_arm_usy = 0.0; 
% ff_qd_d.r_arm_shx = 0.0; 
% ff_qd_d.r_arm_ely = 0.0; 
% ff_qd_d.r_arm_elx = 0.0; 
% ff_qd_d.r_arm_uwy = 0.0; 
% ff_qd_d.r_arm_mwx = 0.0; 
ff_qd_d.l_leg_hpz = 2.0; % 02-03-14, f+v
ff_qd_d.l_leg_hpx = 2.0; % 02-03-14, f+v 
ff_qd_d.l_leg_hpy = 2.0; % 02-03-14, f+v
ff_qd_d.l_leg_kny = 2.0; % 02-03-14, f+v
ff_qd_d.l_leg_aky = 40.0; % 02-03-14, f+v
ff_qd_d.l_leg_akx = 40.0; % 02-03-14, f+v
ff_qd_d.r_leg_hpz = 2.0; % 02-03-14, f+v
ff_qd_d.r_leg_hpx = 2.0; % 02-03-14, f+v
ff_qd_d.r_leg_hpy = 2.0; % 02-03-14, f+v 
ff_qd_d.r_leg_kny = 2.0; % 02-03-14, f+v
ff_qd_d.r_leg_aky = 40.0; % 02-03-14, f+v
ff_qd_d.r_leg_akx = 40.0; % 02-03-14, f+v




gains.k_f_p = double(k_f_p);
gains.k_q_p = double(k_q_p);
gains.k_q_i = double(k_q_i);
gains.k_qd_p = double(k_qd_p);
gains.ff_f_d = double(ff_f_d);
gains.ff_const = double(ff_const);
gains.ff_qd = double(ff_qd);
gains.ff_qd_d = double(ff_qd_d);

end