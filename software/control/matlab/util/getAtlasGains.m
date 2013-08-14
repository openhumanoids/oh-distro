function gains = getAtlasGains(r, mode)
%GETATLASGAINS Returns default Atlas control gains

% mode==1: torque 
% mode==2: position

if mode < 1 || mode > 2
  error('only torque and position control modes supported currently');
end

nq = 28; % fixed number
input_frame = getInputFrame(r);

if input_frame.dim ~= nq
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
  k_f_p = Point(input_frame);
  
  k_f_p.back_bkz_motor = 0.012;
  k_f_p.back_bky_motor = 0.012;
  k_f_p.back_bkx_motor = 0.012;
  
  k_f_p.neck_ay_motor = 0.012;

  k_f_p.l_leg_hpz_motor = 0.012;
  k_f_p.l_leg_hpx_motor = 0.012;
  k_f_p.l_leg_hpy_motor = 0.012;
  k_f_p.l_leg_kny_motor = 0.012;
  k_f_p.l_leg_aky_motor = 1.4;
  k_f_p.l_leg_akx_motor = 1.4;
  
  % mirror left/right (note, this could potentially change)
  k_f_p.r_leg_hpz_motor = k_f_p.l_leg_hpz_motor;
  k_f_p.r_leg_hpx_motor = k_f_p.l_leg_hpx_motor;
  k_f_p.r_leg_hpy_motor = k_f_p.l_leg_hpy_motor;
  k_f_p.r_leg_kny_motor = k_f_p.l_leg_kny_motor;
  k_f_p.r_leg_aky_motor = k_f_p.l_leg_aky_motor;
  k_f_p.r_leg_akx_motor = k_f_p.l_leg_akx_motor;

  k_f_p.l_arm_usy_motor = 0.012;
  k_f_p.l_arm_shx_motor = 0.012;
  k_f_p.l_arm_ely_motor = 0.012;
  k_f_p.l_arm_elx_motor = 0.012;
  k_f_p.l_arm_uwy_motor = 0.012;
  k_f_p.l_arm_mwx_motor = 0.012;

  % mirror left/right (note, this could potentially change)
  k_f_p.r_arm_usy_motor = k_f_p.l_arm_usy_motor;
  k_f_p.r_arm_shx_motor = k_f_p.l_arm_shx_motor;
  k_f_p.r_arm_ely_motor = k_f_p.l_arm_ely_motor;
  k_f_p.r_arm_elx_motor = k_f_p.l_arm_elx_motor;
  k_f_p.r_arm_uwy_motor = k_f_p.l_arm_uwy_motor;
  k_f_p.r_arm_mwx_motor = k_f_p.l_arm_mwx_motor;

  gains.k_f_p_motor = double(k_f_p);
  
elseif mode==2 % return position control gains
  k_q_p = Point(input_frame);
  
  k_q_p.back_bkz_motor = 15.0;
  k_q_p.back_bky_motor = 60.0;
  k_q_p.back_bkx_motor = 60.0;
  
  k_q_p.neck_ay_motor = 5.0;

  k_q_p.l_leg_hpz_motor = 45.0;
  k_q_p.l_leg_hpx_motor = 30.0;
  k_q_p.l_leg_hpy_motor = 60.0;
  k_q_p.l_leg_kny_motor = 60.0;
  k_q_p.l_leg_aky_motor = 2000.0;
  k_q_p.l_leg_akx_motor = 2000.0;
  
  % mirror left/right (note, this could potentially change)
  k_q_p.r_leg_hpz_motor = k_q_p.l_leg_hpz_motor;
  k_q_p.r_leg_hpx_motor = k_q_p.l_leg_hpx_motor;
  k_q_p.r_leg_hpy_motor = k_q_p.l_leg_hpy_motor;
  k_q_p.r_leg_kny_motor = k_q_p.l_leg_kny_motor;
  k_q_p.r_leg_aky_motor = k_q_p.l_leg_aky_motor;
  k_q_p.r_leg_akx_motor = k_q_p.l_leg_akx_motor;

  k_q_p.l_arm_usy_motor = 4.0;
  k_q_p.l_arm_shx_motor = 4.0;
  k_q_p.l_arm_ely_motor = 4.0;
  k_q_p.l_arm_elx_motor = 4.0;
  k_q_p.l_arm_uwy_motor = 4.0;
  k_q_p.l_arm_mwx_motor = 4.0;

  % mirror left/right (note, this could potentially change)
  k_q_p.r_arm_usy_motor = k_q_p.l_arm_usy_motor;
  k_q_p.r_arm_shx_motor = k_q_p.l_arm_shx_motor;
  k_q_p.r_arm_ely_motor = k_q_p.l_arm_ely_motor;
  k_q_p.r_arm_elx_motor = k_q_p.l_arm_elx_motor;
  k_q_p.r_arm_uwy_motor = k_q_p.l_arm_uwy_motor;
  k_q_p.r_arm_mwx_motor = k_q_p.l_arm_mwx_motor;

  gains.k_q_p = double(k_q_p);

end
end

