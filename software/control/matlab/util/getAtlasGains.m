function gains = getAtlasGains(atlas_version)
%GETATLASGAINS Returns default Atlas control gains
if nargin < 1 || isempty(atlas_version), atlas_version = 4; end

switch atlas_version
  case 3
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
    % BDI DEFAULT -----------
    % k_q_p(back_bkz)  = 20.0;
    % k_q_p(back_bky)  = 60.0;
    % k_q_p(back_bkx)  = 60.0;
    % k_q_p(neck_ay)   = 8.0;
    k_q_p(l_leg_hpz) = 45.0;
    k_q_p(l_leg_hpx) = 30.0;
    k_q_p(l_leg_hpy) = 50.0;
    k_q_p(l_leg_kny) = 30.0;
    k_q_p(l_leg_aky) = 1000.0;
    k_q_p(l_leg_akx) = 1000.0;
    % k_q_p(l_arm_usy) = 4.0; 
    % k_q_p(l_arm_shx) = 4.0;  
    % k_q_p(l_arm_ely) = 4.0; 
    % k_q_p(l_arm_elx) = 4.0; 
    % k_q_p(l_arm_uwy) = 4.0; 
    % k_q_p(l_arm_mwx) = 4.0; 
    k_q_p(r_leg_hpz) = k_q_p(l_leg_hpz);
    k_q_p(r_leg_hpx) = k_q_p(l_leg_hpx);
    k_q_p(r_leg_hpy) = k_q_p(l_leg_hpy);
    k_q_p(r_leg_kny) = k_q_p(l_leg_kny);
    k_q_p(r_leg_aky) = k_q_p(l_leg_aky);
    k_q_p(r_leg_akx) = k_q_p(l_leg_akx);
    % k_q_p(r_arm_usy) = 4.0; 
    % k_q_p(r_arm_shx) = 4.0;  
    % k_q_p(r_arm_ely) = 4.0; 
    % k_q_p(r_arm_elx) = 4.0; 
    % k_q_p(r_arm_uwy) = 4.0; 
    % k_q_p(r_arm_mwx) = 4.0; 

    % Trials values -----------
    % arm gains are meant to be slightly overdamped
    k_q_p(back_bkz)  = 20.0;
    k_q_p(back_bky)  = 60.0;
    k_q_p(back_bkx)  = 60.0;
    k_q_p(neck_ay)   = 8.0;
    % k_q_p(l_leg_hpz) = 45.0;% * 0.1;
    % k_q_p(l_leg_hpx) = 45.0;% * 0.1;
    % k_q_p(l_leg_hpy) = 55.0;% * 0.1;
    % k_q_p(l_leg_kny) = 60.0;% * 0.1;
    % k_q_p(l_leg_aky) = 1000.0;% * 0.1;
    % k_q_p(l_leg_akx) = 1000.0;% * 0.1;
    k_q_p(l_arm_usy) = 10.5;
    k_q_p(l_arm_shx) = 10.0; 
    k_q_p(l_arm_ely) = 15.0; 
    k_q_p(l_arm_elx) = 30.0;
    k_q_p(l_arm_uwy) = 10.0; 
    k_q_p(l_arm_mwx) = 10.0; 
    % k_q_p(r_leg_hpz) = k_q_p(l_leg_hpz);
    % k_q_p(r_leg_hpx) = k_q_p(l_leg_hpx);
    % k_q_p(r_leg_hpy) = k_q_p(l_leg_hpy);
    % k_q_p(r_leg_kny) = k_q_p(l_leg_kny);
    % k_q_p(r_leg_aky) = k_q_p(l_leg_aky);
    % k_q_p(r_leg_akx) = k_q_p(l_leg_akx);
    k_q_p(r_arm_usy) = 19.0;
    k_q_p(r_arm_shx) = 10.0;  
    k_q_p(r_arm_ely) = 12.0;
    k_q_p(r_arm_elx) = 20.0; 
    k_q_p(r_arm_uwy) = 16.0;
    k_q_p(r_arm_mwx) = 14.0; 


    % velocity, proportunal
    % k_qd_p(back_bkz)  = 0.5;
    % k_qd_p(back_bky)  = 1.0;
    % k_qd_p(back_bkx)  = 1.0;
    % k_qd_p(neck_ay)   = 0.1;

    % Trials values -------
    k_qd_p(back_bkz)  = 0.85;
    k_qd_p(back_bky)  = 5.0;
    k_qd_p(back_bkx)  = 5.0;
    k_qd_p(neck_ay)   = 0.1;
    % k_qd_p(l_leg_hpz) = 0.1;
    % k_qd_p(l_leg_hpx) = 0.1;
    % k_qd_p(l_leg_hpy) = 0.2;
    % k_qd_p(l_leg_kny) = 0.2;
    % k_qd_p(l_leg_aky) = 2.5;
    % k_qd_p(l_leg_akx) = 0.1;
    k_qd_p(l_arm_usy) = 0.85; 
    k_qd_p(l_arm_shx) = 1.25;  
    k_qd_p(l_arm_ely) = 0.25; 
    k_qd_p(l_arm_elx) = 1.75;
    k_qd_p(l_arm_uwy) = 0.05; 
    k_qd_p(l_arm_mwx) = 0.25; 
    % k_qd_p(r_leg_hpz) = k_qd_p(l_leg_hpz);
    % k_qd_p(r_leg_hpx) = k_qd_p(l_leg_hpx);
    % k_qd_p(r_leg_hpy) = k_qd_p(l_leg_hpy);
    % k_qd_p(r_leg_kny) = k_qd_p(l_leg_kny);
    % k_qd_p(r_leg_aky) = k_qd_p(l_leg_aky);
    % k_qd_p(r_leg_akx) = k_qd_p(l_leg_akx);
    k_qd_p(r_arm_usy) = 1.5;
    k_qd_p(r_arm_shx) = 1.25;
    k_qd_p(r_arm_ely) = 0.25; 
    k_qd_p(r_arm_elx) = 0.5; 
    k_qd_p(r_arm_uwy) = 0.25; 
    k_qd_p(r_arm_mwx) = 0.25;



    % force, proportunal
    k_f_p(back_bkz)  = 0.005;
    k_f_p(back_bky)  = 0.02;
    k_f_p(back_bkx)  = 0.02;
    k_f_p(l_leg_hpz) = 0.02; % 02-03-14, f+v 
    k_f_p(l_leg_hpx) = 0.02; % 02-03-14, f+v 
    k_f_p(l_leg_hpy) = 0.02; % 02-03-14, f+v
    k_f_p(l_leg_kny) = 0.02; % 02-03-14, f+v
    k_f_p(l_leg_aky) = 0.45; % 02-03-14, f+v
    k_f_p(l_leg_akx) = 0.75; % 02-03-14, f+v 
    k_f_p(r_leg_hpz) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_hpx) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_hpy) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_kny) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_aky) = 0.45; % 02-03-14, f+v
    k_f_p(r_leg_akx) = 0.75; % 02-03-14, f+v 
    k_f_p(l_arm_usy) = 0.08; % 9-17-13
    k_f_p(l_arm_shx) = 0.125; % 9-17-13
    k_f_p(l_arm_ely) = 0.115; % 9-17-13
    k_f_p(l_arm_elx) = 0.135; % 9-17-13
    k_f_p(l_arm_uwy) = 0.085; % 9-17-13
    k_f_p(l_arm_mwx) = 0.125; % 9-17-13
    k_f_p(r_arm_usy) = 0.09; % 10-09-13
    k_f_p(r_arm_shx) = 0.125; % 9-17-13
    k_f_p(r_arm_ely) = 0.125; % 9-17-13
    k_f_p(r_arm_elx) = 0.125; % 9-17-13
    k_f_p(r_arm_uwy) = 0.085; % 9-17-13
    k_f_p(r_arm_mwx) = 0.125; % 9-17-13

    % velocity desired, feedforward
    ff_qd_d(back_bkz)  = 1.0;
    ff_qd_d(back_bky)  = 3.0;
    ff_qd_d(back_bkx)  = 3.0;
    ff_qd_d(l_leg_hpz) = 1.0; % 03-24-14, f+v 
    ff_qd_d(l_leg_hpx) = 4.0; % 03-24-14, f+v 
    ff_qd_d(l_leg_hpy) = 4.0; % 03-24-14, f+v
    ff_qd_d(l_leg_kny) = 4.0; % 03-24-14, f+v
    ff_qd_d(l_leg_aky) = 100.0; 
    ff_qd_d(l_leg_akx) = 100.0; 
    ff_qd_d(r_leg_hpz) = 1.0; % 03-24-14, f+v 
    ff_qd_d(r_leg_hpx) = 4.0; % 03-24-14, f+v  
    ff_qd_d(r_leg_hpy) = 4.0; % 03-24-14, f+v 
    ff_qd_d(r_leg_kny) = 4.0; % 03-24-14, f+v
    ff_qd_d(r_leg_aky) = 100.0; 
    ff_qd_d(r_leg_akx) = 100.0; 

    % velocity, feedforward
    ff_qd(l_arm_usy) = 0.3; % 9-19-13, fc
    ff_qd(r_arm_usy) = 0.3; % 9-19-13, fc
    ff_qd(l_arm_shx) = 0.275; % 9-19-13, fc
    ff_qd(r_arm_shx) = 0.275; % 9-18-13, fc
    ff_qd(l_arm_ely) = 0.25; % 9-19-13, fc
    ff_qd(r_arm_ely) = 0.25; % 9-18-13, fc
    ff_qd(l_arm_elx) = 0.3; % 9-19-13, fc
    ff_qd(r_arm_elx) = 0.25; % 9-18-13, fc
    ff_qd(l_arm_uwy) = 0.22; % 9-19-13, fc
    ff_qd(r_arm_uwy) = 0.22; % 9-18-13, fc
    ff_qd(l_arm_mwx) = 0.225; % 9-19-13, fc
    ff_qd(r_arm_mwx) = 0.225; % 9-19-13, fc
  case 4
    % fixed input frame ordering
    back_bkz  = 1;
    back_bky  = 2;
    back_bkx  = 3;
    l_arm_elx = 4;
    l_arm_ely = 5;
    l_arm_mwx = 6;
    l_arm_shx = 7;
    l_arm_shz = 8;
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
    r_arm_shz = 21;
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
    % BDI DEFAULT -----------
    % k_q_p(back_bkz)  = 10.0;
    % k_q_p(back_bky)  = 70.0;
    % k_q_p(back_bkx)  = 70.0;
    % k_q_p(neck_ay)   = 1000.0;
    k_q_p(l_leg_hpz) = 45.0;
    k_q_p(l_leg_hpx) = 30.0;
    k_q_p(l_leg_hpy) = 60.0;
    k_q_p(l_leg_kny) = 60.0;
    k_q_p(l_leg_aky) = 2000.0;
    k_q_p(l_leg_akx) = 2000.0;
    % k_q_p(l_arm_shz) = 4.0; 
    % k_q_p(l_arm_shx) = 4.0;  
    % k_q_p(l_arm_ely) = 4.0; 
    % k_q_p(l_arm_elx) = 4.0; 
    % k_q_p(l_arm_uwy) = 8.0; 
    % k_q_p(l_arm_mwx) = 8.0; 
    k_q_p(r_leg_hpz) = k_q_p(l_leg_hpz);
    k_q_p(r_leg_hpx) = k_q_p(l_leg_hpx);
    k_q_p(r_leg_hpy) = k_q_p(l_leg_hpy);
    k_q_p(r_leg_kny) = k_q_p(l_leg_kny);
    k_q_p(r_leg_aky) = k_q_p(l_leg_aky);
    k_q_p(r_leg_akx) = k_q_p(l_leg_akx);
    % k_q_p(r_arm_shz) = 4.0; 
    % k_q_p(r_arm_shx) = 4.0;  
    % k_q_p(r_arm_ely) = 4.0; 
    % k_q_p(r_arm_elx) = 4.0; 
    % k_q_p(r_arm_uwy) = 8.0; 
    % k_q_p(r_arm_mwx) = 8.0; 

    % Trials values -----------
    % arm gains are meant to be slightly overdamped
    k_q_p(back_bkz)  = 20.0;
    k_q_p(back_bky)  = 60.0;
    k_q_p(back_bkx)  = 60.0;
    k_q_p(neck_ay)   = 8.0;
    % k_q_p(l_leg_hpz) = 45.0;% * 0.1;
    % k_q_p(l_leg_hpx) = 45.0;% * 0.1;
    % k_q_p(l_leg_hpy) = 55.0;% * 0.1;
    % k_q_p(l_leg_kny) = 60.0;% * 0.1;
    % k_q_p(l_leg_aky) = 1000.0;% * 0.1;
    % k_q_p(l_leg_akx) = 1000.0;% * 0.1;
    k_q_p(l_arm_shz) = 10.5;
    k_q_p(l_arm_shx) = 10.0; 
    k_q_p(l_arm_ely) = 15.0; 
    k_q_p(l_arm_elx) = 30.0;
    k_q_p(l_arm_uwy) = 10.0; 
    k_q_p(l_arm_mwx) = 10.0; 
    % k_q_p(r_leg_hpz) = k_q_p(l_leg_hpz);
    % k_q_p(r_leg_hpx) = k_q_p(l_leg_hpx);
    % k_q_p(r_leg_hpy) = k_q_p(l_leg_hpy);
    % k_q_p(r_leg_kny) = k_q_p(l_leg_kny);
    % k_q_p(r_leg_aky) = k_q_p(l_leg_aky);
    % k_q_p(r_leg_akx) = k_q_p(l_leg_akx);
    k_q_p(r_arm_shz) = 19.0;
    k_q_p(r_arm_shx) = 10.0;  
    k_q_p(r_arm_ely) = 12.0;
    k_q_p(r_arm_elx) = 20.0; 
    k_q_p(r_arm_uwy) = 16.0;
    k_q_p(r_arm_mwx) = 14.0; 


    % velocity, proportunal
    % k_qd_p(back_bkz)  = 0.5;
    % k_qd_p(back_bky)  = 1.0;
    % k_qd_p(back_bkx)  = 1.0;
    % k_qd_p(neck_ay)   = 0.1;

    % Trials values -------
    k_qd_p(back_bkz)  = 0.85;
    k_qd_p(back_bky)  = 5.0;
    k_qd_p(back_bkx)  = 5.0;
    k_qd_p(neck_ay)   = 0.1;
    % k_qd_p(l_leg_hpz) = 0.1;
    % k_qd_p(l_leg_hpx) = 0.1;
    % k_qd_p(l_leg_hpy) = 0.2;
    % k_qd_p(l_leg_kny) = 0.2;
    % k_qd_p(l_leg_aky) = 2.5;
    % k_qd_p(l_leg_akx) = 0.1;
    k_qd_p(l_arm_shz) = 0.85; 
    k_qd_p(l_arm_shx) = 1.25;  
    k_qd_p(l_arm_ely) = 0.25; 
    k_qd_p(l_arm_elx) = 1.75;
    k_qd_p(l_arm_uwy) = 0.05; 
    k_qd_p(l_arm_mwx) = 0.25; 
    % k_qd_p(r_leg_hpz) = k_qd_p(l_leg_hpz);
    % k_qd_p(r_leg_hpx) = k_qd_p(l_leg_hpx);
    % k_qd_p(r_leg_hpy) = k_qd_p(l_leg_hpy);
    % k_qd_p(r_leg_kny) = k_qd_p(l_leg_kny);
    % k_qd_p(r_leg_aky) = k_qd_p(l_leg_aky);
    % k_qd_p(r_leg_akx) = k_qd_p(l_leg_akx);
    k_qd_p(r_arm_shz) = 1.5;
    k_qd_p(r_arm_shx) = 1.25;
    k_qd_p(r_arm_ely) = 0.25; 
    k_qd_p(r_arm_elx) = 0.5; 
    k_qd_p(r_arm_uwy) = 0.25; 
    k_qd_p(r_arm_mwx) = 0.25;



    % force, proportunal
    k_f_p(back_bkz)  = 0.005;
    k_f_p(back_bky)  = 0.02;
    k_f_p(back_bkx)  = 0.0025;
    k_f_p(l_leg_hpz) = 0.02; % 02-03-14, f+v 
    k_f_p(l_leg_hpx) = 0.02; % 02-03-14, f+v 
    k_f_p(l_leg_hpy) = 0.02; % 02-03-14, f+v
    k_f_p(l_leg_kny) = 0.02; % 02-03-14, f+v
    k_f_p(l_leg_aky) = 0.35; % 02-03-14, f+v
    k_f_p(l_leg_akx) = 0.75; % 02-03-14, f+v 
    k_f_p(r_leg_hpz) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_hpx) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_hpy) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_kny) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_aky) = 0.35; % 02-03-14, f+v
    k_f_p(r_leg_akx) = 0.75; % 02-03-14, f+v 
    k_f_p(l_arm_shz) = 0.08; % 9-17-13
    k_f_p(l_arm_shx) = 0.125; % 9-17-13
    k_f_p(l_arm_ely) = 0.115; % 9-17-13
    k_f_p(l_arm_elx) = 0.135; % 9-17-13
    k_f_p(l_arm_uwy) = 0.085; % 9-17-13
    k_f_p(l_arm_mwx) = 0.125; % 9-17-13
    k_f_p(r_arm_shz) = 0.09; % 10-09-13
    k_f_p(r_arm_shx) = 0.125; % 9-17-13
    k_f_p(r_arm_ely) = 0.125; % 9-17-13
    k_f_p(r_arm_elx) = 0.125; % 9-17-13
    k_f_p(r_arm_uwy) = 0.085; % 9-17-13
    k_f_p(r_arm_mwx) = 0.125; % 9-17-13

    % velocity desired, feedforward
    ff_qd_d(back_bkz)  = 1.0;
    ff_qd_d(back_bky)  = 3.0;
    ff_qd_d(back_bkx)  = 1.0;
    ff_qd_d(l_leg_hpz) = 1.0; % 03-24-14, f+v 
    ff_qd_d(l_leg_hpx) = 4.0; % 03-24-14, f+v 
    ff_qd_d(l_leg_hpy) = 5.25;
    ff_qd_d(l_leg_kny) = 5.5; % 03-24-14, f+v
    ff_qd_d(l_leg_aky) = 100.0; 
    ff_qd_d(l_leg_akx) = 100.0; 
    ff_qd_d(r_leg_hpz) = 1.0; % 03-24-14, f+v 
    ff_qd_d(r_leg_hpx) = 4.0; % 03-24-14, f+v  
    ff_qd_d(r_leg_hpy) = 5.25; 
    ff_qd_d(r_leg_kny) = 5.5; % 03-24-14, f+v
    ff_qd_d(r_leg_aky) = 100.0; 
    ff_qd_d(r_leg_akx) = 100.0; 

    % % velocity, feedforward
    % ff_qd(l_arm_shz) = 0.3; % 9-19-13, fc
    % ff_qd(r_arm_shz) = 0.3; % 9-19-13, fc
    % ff_qd(l_arm_shx) = 0.275; % 9-19-13, fc
    % ff_qd(r_arm_shx) = 0.275; % 9-18-13, fc
    % ff_qd(l_arm_ely) = 0.25; % 9-19-13, fc
    % ff_qd(r_arm_ely) = 0.25; % 9-18-13, fc
    % ff_qd(l_arm_elx) = 0.3; % 9-19-13, fc
    % ff_qd(r_arm_elx) = 0.25; % 9-18-13, fc
    % ff_qd(l_arm_uwy) = 0.22; % 9-19-13, fc
    % ff_qd(r_arm_uwy) = 0.22; % 9-18-13, fc
    % ff_qd(l_arm_mwx) = 0.225; % 9-19-13, fc
    % ff_qd(r_arm_mwx) = 0.225; % 9-19-13, fc
  case 5
    % fixed input frame ordering
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

    nu = 30;

    k_f_p    = zeros(nu,1);
    k_q_p    = zeros(nu,1);
    k_q_i    = zeros(nu,1);
    k_qd_p   = zeros(nu,1);
    ff_qd    = zeros(nu,1);
    ff_f_d   = zeros(nu,1);
    ff_const = zeros(nu,1);
    ff_qd_d  = zeros(nu,1);

    % position, proportunal
    % BDI DEFAULT -----------
    % k_q_p(back_bkz)  = 10.0;
    % k_q_p(back_bky)  = 70.0;
    % k_q_p(back_bkx)  = 70.0;
    % k_q_p(neck_ay)   = 1000.0;
    k_q_p(l_leg_hpz) = 45.0;
    k_q_p(l_leg_hpx) = 30.0;
    k_q_p(l_leg_hpy) = 60.0;
    k_q_p(l_leg_kny) = 60.0;
    k_q_p(l_leg_aky) = 2000.0;
    k_q_p(l_leg_akx) = 2000.0;
    % k_q_p(l_arm_shz) = 4.0; 
    % k_q_p(l_arm_shx) = 4.0;  
    % k_q_p(l_arm_ely) = 4.0; 
    % k_q_p(l_arm_elx) = 4.0; 
    % k_q_p(l_arm_uwy) = 8.0; 
    % k_q_p(l_arm_mwx) = 8.0; 
    k_q_p(r_leg_hpz) = k_q_p(l_leg_hpz);
    k_q_p(r_leg_hpx) = k_q_p(l_leg_hpx);
    k_q_p(r_leg_hpy) = k_q_p(l_leg_hpy);
    k_q_p(r_leg_kny) = k_q_p(l_leg_kny);
    k_q_p(r_leg_aky) = k_q_p(l_leg_aky);
    k_q_p(r_leg_akx) = k_q_p(l_leg_akx);
    % k_q_p(r_arm_shz) = 4.0; 
    % k_q_p(r_arm_shx) = 4.0;  
    % k_q_p(r_arm_ely) = 4.0; 
    % k_q_p(r_arm_elx) = 4.0; 
    % k_q_p(r_arm_uwy) = 8.0; 
    % k_q_p(r_arm_mwx) = 8.0; 

    % Trials values -----------
    % arm gains are meant to be slightly overdamped
    k_q_p(back_bkz)  = 20.0;
    k_q_p(back_bky)  = 60.0;
    k_q_p(back_bkx)  = 60.0;
    k_q_p(neck_ay)   = 8.0;
    % k_q_p(l_leg_hpz) = 45.0;% * 0.1;
    % k_q_p(l_leg_hpx) = 45.0;% * 0.1;
    % k_q_p(l_leg_hpy) = 55.0;% * 0.1;
    % k_q_p(l_leg_kny) = 60.0;% * 0.1;
    % k_q_p(l_leg_aky) = 1000.0;% * 0.1;
    % k_q_p(l_leg_akx) = 1000.0;% * 0.1;
    k_q_p(l_arm_shz) = 10.5;
    k_q_p(l_arm_shx) = 10.0; 
    k_q_p(l_arm_ely) = 15.0; 
    k_q_p(l_arm_elx) = 30.0;
    k_q_p(l_arm_uwy) = 20.0; 
    k_q_p(l_arm_mwx) = 20.0; 
    k_q_p(l_arm_lwy) = 20.0; 
    
    % k_q_p(r_leg_hpz) = k_q_p(l_leg_hpz);
    % k_q_p(r_leg_hpx) = k_q_p(l_leg_hpx);
    % k_q_p(r_leg_hpy) = k_q_p(l_leg_hpy);
    % k_q_p(r_leg_kny) = k_q_p(l_leg_kny);
    % k_q_p(r_leg_aky) = k_q_p(l_leg_aky);
    % k_q_p(r_leg_akx) = k_q_p(l_leg_akx);
    k_q_p(r_arm_shz) = 19.0;
    k_q_p(r_arm_shx) = 10.0;  
    k_q_p(r_arm_ely) = 12.0;
    k_q_p(r_arm_elx) = 20.0; 
    k_q_p(r_arm_uwy) = 20.0;
    k_q_p(r_arm_mwx) = 20.0; 
    k_q_p(r_arm_lwy) = 20.0; 


    % velocity, proportunal
    % k_qd_p(back_bkz)  = 0.5;
    % k_qd_p(back_bky)  = 1.0;
    % k_qd_p(back_bkx)  = 1.0;
    % k_qd_p(neck_ay)   = 0.1;

    % Trials values -------
    k_qd_p(back_bkz)  = 1.95;
    k_qd_p(back_bky)  = 5.0;
    k_qd_p(back_bkx)  = 5.0;
    k_qd_p(neck_ay)   = 0.1;
    % k_qd_p(l_leg_hpz) = 0.1;
    % k_qd_p(l_leg_hpx) = 0.1;
    % k_qd_p(l_leg_hpy) = 0.2;
    % k_qd_p(l_leg_kny) = 0.2;
    % k_qd_p(l_leg_aky) = 2.5;
    % k_qd_p(l_leg_akx) = 0.1;
    k_qd_p(l_arm_shz) = 0.85; 
    k_qd_p(l_arm_shx) = 1.25;  
    k_qd_p(l_arm_ely) = 0.25; 
    k_qd_p(l_arm_elx) = 1.75;
    k_qd_p(l_arm_uwy) = 0.5; 
    k_qd_p(l_arm_mwx) = 0.5; 
    k_qd_p(l_arm_lwy) = 0.5; 
    % k_qd_p(r_leg_hpz) = k_qd_p(l_leg_hpz);
    % k_qd_p(r_leg_hpx) = k_qd_p(l_leg_hpx);
    % k_qd_p(r_leg_hpy) = k_qd_p(l_leg_hpy);
    % k_qd_p(r_leg_kny) = k_qd_p(l_leg_kny);
    % k_qd_p(r_leg_aky) = k_qd_p(l_leg_aky);
    % k_qd_p(r_leg_akx) = k_qd_p(l_leg_akx);
    k_qd_p(r_arm_shz) = 1.5;
    k_qd_p(r_arm_shx) = 1.25;
    k_qd_p(r_arm_ely) = 0.25; 
    k_qd_p(r_arm_elx) = 0.6; 
    k_qd_p(r_arm_uwy) = 0.5; 
    k_qd_p(r_arm_mwx) = 0.5; 
    k_qd_p(r_arm_lwy) = 0.5; 



    % force, proportunal
    k_f_p(back_bkz)  = 0.005;
    k_f_p(back_bky)  = 0.02;
    k_f_p(back_bkx)  = 0.0025;
    k_f_p(l_leg_hpz) = 0.02; % 02-03-14, f+v 
    k_f_p(l_leg_hpx) = 0.02; % 02-03-14, f+v 
    k_f_p(l_leg_hpy) = 0.02; % 02-03-14, f+v
    k_f_p(l_leg_kny) = 0.02; % 02-03-14, f+v
    k_f_p(l_leg_aky) = 0.35; % 02-03-14, f+v
    k_f_p(l_leg_akx) = 0.75; % 02-03-14, f+v 
    k_f_p(r_leg_hpz) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_hpx) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_hpy) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_kny) = 0.02; % 02-03-14, f+v 
    k_f_p(r_leg_aky) = 0.35; % 02-03-14, f+v
    k_f_p(r_leg_akx) = 0.75; % 02-03-14, f+v 
    k_f_p(l_arm_shz) = 0.08; % 9-17-13
    k_f_p(l_arm_shx) = 0.125; % 9-17-13
    k_f_p(l_arm_ely) = 0.115; % 9-17-13
    k_f_p(l_arm_elx) = 0.135; % 9-17-13
    k_f_p(l_arm_uwy) = 0.085; % 9-17-13
    k_f_p(l_arm_mwx) = 0.125; % 9-17-13
    k_f_p(r_arm_shz) = 0.09; % 10-09-13
    k_f_p(r_arm_shx) = 0.125; % 9-17-13
    k_f_p(r_arm_ely) = 0.125; % 9-17-13
    k_f_p(r_arm_elx) = 0.125; % 9-17-13
    k_f_p(r_arm_uwy) = 0.085; % 9-17-13
    k_f_p(r_arm_mwx) = 0.125; % 9-17-13

    % velocity desired, feedforward
    ff_qd_d(back_bkz)  = 1.0;
    ff_qd_d(back_bky)  = 3.0;
    ff_qd_d(back_bkx)  = 1.0;
    ff_qd_d(l_leg_hpz) = 1.0; % 03-24-14, f+v 
    ff_qd_d(l_leg_hpx) = 2.5; % 03-24-14, f+v 
    ff_qd_d(l_leg_hpy) = 4.0;
    ff_qd_d(l_leg_kny) = 4.0; % 03-24-14, f+v
    ff_qd_d(l_leg_aky) = 100.0; 
    ff_qd_d(l_leg_akx) = 100.0; 
    ff_qd_d(r_leg_hpz) = 1.0; % 03-24-14, f+v 
    ff_qd_d(r_leg_hpx) = 2.5; % 03-24-14, f+v  
    ff_qd_d(r_leg_hpy) = 4.0; 
    ff_qd_d(r_leg_kny) = 4.0; % 03-24-14, f+v
    ff_qd_d(r_leg_aky) = 100.0; 
    ff_qd_d(r_leg_akx) = 100.0; 


  otherwise
    error('getAtlasGains:BadAtlasVersion', ...
      'Invalid Atlas version. Valid values are 3, 4, and 5');
end


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
