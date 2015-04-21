function params = getHardwareParams(r)
typecheck(r, 'DRCAtlas');

force_controlled_joints = [];
nu = r.getNumInputs();
position_controlled_joints = (1:nu)';

gains.k_f_p    = zeros(nu,1);
gains.k_q_p    = zeros(nu,1);
gains.k_q_i    = zeros(nu,1);
gains.k_qd_p   = zeros(nu,1);
gains.ff_qd    = zeros(nu,1);
gains.ff_f_d   = zeros(nu,1);
gains.ff_const = zeros(nu,1);
gains.ff_qd_d  = zeros(nu,1);

back_bkz   = 1;
back_bky   = 2;
back_bkx   = 3;
l_arm_elx  = 4;
l_arm_ely  = 5;
l_arm_mwx  = 6;
l_arm_shx  = 7;
l_arm_shz  = 8;
l_arm_uwy  = 9;
l_arm_lwy = 10;
l_leg_kny  = 11;
l_leg_akx  = 12;
l_leg_hpy  = 13;
l_leg_hpx  = 14;
l_leg_aky  = 15;
l_leg_hpz  = 16;
neck_ay    = 17;
r_arm_elx  = 18;
r_arm_ely  = 19;
r_arm_mwx  = 20;
r_arm_shx  = 21;
r_arm_shz  = 22;
r_arm_uwy  = 23;
r_arm_lwy = 24;
r_leg_kny  = 25;
r_leg_akx  = 26;
r_leg_hpy  = 27;
r_leg_hpx  = 28;
r_leg_aky  = 29;
r_leg_hpz  = 30;

% position, proportunal
gains.k_q_p(back_bkz)  = 10.0;
gains.k_q_p(back_bky)  = 25.0;
gains.k_q_p(back_bkx)  = 25.0;
gains.k_q_p(l_leg_hpz) = 20.0;
gains.k_q_p(l_leg_hpx) = 15.0;
gains.k_q_p(l_leg_hpy) = 30.0;
gains.k_q_p(l_leg_kny) = 30.0;
gains.k_q_p(l_leg_aky) = 500.0;
gains.k_q_p(l_leg_akx) = 500.0;
gains.k_q_p(l_arm_shz) = 5.0;
gains.k_q_p(l_arm_shx) = 5.0; 
gains.k_q_p(l_arm_ely) = 5.0; 
gains.k_q_p(l_arm_elx) = 15.0;
gains.k_q_p(l_arm_uwy) = 10; 
gains.k_q_p(l_arm_mwx) = 10.0; 
gains.k_q_p(l_arm_lwy) = 10.0; 
gains.k_q_p(r_leg_hpz) = gains.k_q_p(l_leg_hpz);
gains.k_q_p(r_leg_hpx) = gains.k_q_p(l_leg_hpx);
gains.k_q_p(r_leg_hpy) = gains.k_q_p(l_leg_hpy);
gains.k_q_p(r_leg_kny) = gains.k_q_p(l_leg_kny);
gains.k_q_p(r_leg_aky) = gains.k_q_p(l_leg_aky);
gains.k_q_p(r_leg_akx) = gains.k_q_p(l_leg_akx);
gains.k_q_p(r_arm_shz) = gains.k_q_p(l_arm_shz);
gains.k_q_p(r_arm_shx) = gains.k_q_p(l_arm_shx);  
gains.k_q_p(r_arm_ely) = gains.k_q_p(l_arm_ely);
gains.k_q_p(r_arm_elx) = gains.k_q_p(l_arm_elx); 
gains.k_q_p(r_arm_uwy) = gains.k_q_p(l_arm_uwy);
gains.k_q_p(r_arm_mwx) = gains.k_q_p(l_arm_mwx); 
gains.k_q_p(r_arm_lwy) = gains.k_q_p(l_arm_lwy); 


% velocity, proportunal
gains.k_qd_p(back_bkz)  = 1.0;
gains.k_qd_p(back_bky)  = 2.5;
gains.k_qd_p(back_bkx)  = 2.5;
gains.k_qd_p(l_leg_hpz) = 0.1;
gains.k_qd_p(l_leg_hpx) = 0.1;
gains.k_qd_p(l_leg_hpy) = 0.2;
gains.k_qd_p(l_leg_kny) = 0.2;
gains.k_qd_p(l_leg_aky) = 2.5;
gains.k_qd_p(l_leg_akx) = 0.1;
gains.k_qd_p(l_arm_shz) = 0.45; 
gains.k_qd_p(l_arm_shx) = 0.65;  
gains.k_qd_p(l_arm_ely) = 0.25; 
gains.k_qd_p(l_arm_elx) = 0.75;
gains.k_qd_p(l_arm_uwy) = 0.25; 
gains.k_qd_p(l_arm_mwx) = 0.25; 
gains.k_qd_p(l_arm_lwy) = 0.25; 
gains.k_qd_p(r_leg_hpz) = gains.k_qd_p(l_leg_hpz);
gains.k_qd_p(r_leg_hpx) = gains.k_qd_p(l_leg_hpx);
gains.k_qd_p(r_leg_hpy) = gains.k_qd_p(l_leg_hpy);
gains.k_qd_p(r_leg_kny) = gains.k_qd_p(l_leg_kny);
gains.k_qd_p(r_leg_aky) = gains.k_qd_p(l_leg_aky);
gains.k_qd_p(r_leg_akx) = gains.k_qd_p(l_leg_akx);
gains.k_qd_p(r_arm_shz) = gains.k_qd_p(r_arm_shz);
gains.k_qd_p(r_arm_shx) = gains.k_qd_p(r_arm_shz);
gains.k_qd_p(r_arm_ely) = gains.k_qd_p(r_arm_shz); 
gains.k_qd_p(r_arm_elx) = gains.k_qd_p(r_arm_shz); 
gains.k_qd_p(r_arm_uwy) = gains.k_qd_p(r_arm_shz); 
gains.k_qd_p(r_arm_mwx) = gains.k_qd_p(r_arm_shz); 
gains.k_qd_p(r_arm_lwy) = gains.k_qd_p(r_arm_shz); 

joint_is_force_controlled = zeros(r.getNumInputs(), 1);
joint_is_position_controlled = zeros(r.getNumInputs(), 1);
joint_is_force_controlled(force_controlled_joints) = 1;
joint_is_position_controlled(position_controlled_joints) = 1;

params = struct('gains', gains,...
                'joint_is_force_controlled', joint_is_force_controlled,...
                'joint_is_position_controlled', joint_is_position_controlled);
