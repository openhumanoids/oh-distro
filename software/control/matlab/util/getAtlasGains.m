function gains = getAtlasGains(atlas_input_frame, mode)
%GETATLASGAINS Returns default Atlas control gains

% mode==1: torque 
% mode==2: position

if mode < 1 || mode > 2
  error('only torque and position control modes supported currently');
end

nq = 28; % fixed number

if atlas_input_frame.dim ~= nq
  error('joint_names has incorrect number of elements.');
end

gains = struct();
gains.k_q_p = zeros(nq,1);
gains.k_qd_p = zeros(nq,1);
gains.k_q_i = zeros(nq,1);
gains.k_f_p = zeros(nq,1);
gains.ff_f_d = zeros(nq,1);
gains.ff_qd = zeros(nq,1);
gains.ff_qd_d = zeros(nq,1);
gains.ff_const = zeros(nq,1);

if mode==1 % return torque control gains

  % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  % NOTE: these are the default gains from BDI---they appear to be crap --sk
  % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  k_f_p = Point(atlas_input_frame);
  
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
  
  % mirror left/right (note, this could potentially change)
  k_f_p.r_leg_hpz = k_f_p.l_leg_hpz;
  k_f_p.r_leg_hpx = k_f_p.l_leg_hpx;
  k_f_p.r_leg_hpy = k_f_p.l_leg_hpy;
  k_f_p.r_leg_kny = k_f_p.l_leg_kny;
  k_f_p.r_leg_aky = k_f_p.l_leg_aky;
  k_f_p.r_leg_akx = k_f_p.l_leg_akx;

  k_f_p.l_arm_usy = 0.012;
  k_f_p.l_arm_shx = 0.012;
  k_f_p.l_arm_ely = 0.012;
  k_f_p.l_arm_elx = 0.012;
  k_f_p.l_arm_uwy = 0.012;
  k_f_p.l_arm_mwx = 0.012;

  % mirror left/right (note, this could potentially change)
  k_f_p.r_arm_usy = k_f_p.l_arm_usy;
  k_f_p.r_arm_shx = k_f_p.l_arm_shx;
  k_f_p.r_arm_ely = k_f_p.l_arm_ely;
  k_f_p.r_arm_elx = k_f_p.l_arm_elx;
  k_f_p.r_arm_uwy = k_f_p.l_arm_uwy;
  k_f_p.r_arm_mwx = k_f_p.l_arm_mwx;

  gains.k_f_p = double(k_f_p);
  
elseif mode==2 % return position control gains
  k_q_p = Point(atlas_input_frame);
  
  k_q_p.back_bkz = 15.0;
  k_q_p.back_bky = 60.0;
  k_q_p.back_bkx = 60.0;
  
  k_q_p.neck_ay = 5.0;

  k_q_p.l_leg_hpz = 45.0;
  k_q_p.l_leg_hpx = 30.0;
  k_q_p.l_leg_hpy = 60.0;
  k_q_p.l_leg_kny = 60.0;
  k_q_p.l_leg_aky = 2000.0;
  k_q_p.l_leg_akx = 2000.0;
  
  % mirror left/right (note, this could potentially change)
  k_q_p.r_leg_hpz = k_q_p.l_leg_hpz;
  k_q_p.r_leg_hpx = k_q_p.l_leg_hpx;
  k_q_p.r_leg_hpy = k_q_p.l_leg_hpy;
  k_q_p.r_leg_kny = k_q_p.l_leg_kny;
  k_q_p.r_leg_aky = k_q_p.l_leg_aky;
  k_q_p.r_leg_akx = k_q_p.l_leg_akx;

  k_q_p.l_arm_usy = 4.0;
  k_q_p.l_arm_shx = 4.0;
  k_q_p.l_arm_ely = 4.0;
  k_q_p.l_arm_elx = 4.0;
  k_q_p.l_arm_uwy = 4.0;
  k_q_p.l_arm_mwx = 4.0;

  % mirror left/right (note, this could potentially change)
  k_q_p.r_arm_usy = k_q_p.l_arm_usy;
  k_q_p.r_arm_shx = k_q_p.l_arm_shx;
  k_q_p.r_arm_ely = k_q_p.l_arm_ely;
  k_q_p.r_arm_elx = k_q_p.l_arm_elx;
  k_q_p.r_arm_uwy = k_q_p.l_arm_uwy;
  k_q_p.r_arm_mwx = k_q_p.l_arm_mwx;

  gains.k_q_p = double(k_q_p);

end
end

