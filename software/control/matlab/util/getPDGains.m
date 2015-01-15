function [Kp,Kd] = getPDGains(r,mode)
%NOTEST

if nargin<2
  mode = 'default';
end

fr = getInputFrame(r);
if (isa(fr, 'MultiCoordinateFrame'))
  fr = fr.getFrameByName('drcFrames.AtlasInput');
end
Kp = Point(fr);
Kd = Point(fr);

switch r.atlas_version
  case 3
    if strcmpi(mode,'default')
      Kp.l_arm_usy = 625.0; %%%%%
      Kp.l_arm_shx = 3000.0; %%%%%
      Kp.l_arm_ely = 650.0; %%%%%
      Kp.l_arm_elx = 1600.0; %%%%%s
      Kp.l_arm_uwy = 25.0; %%%%%
      Kp.l_arm_mwx = 375.0; %%%%%
      Kp.l_leg_hpz = 800.0; 
      Kp.l_leg_hpx = 6000.0; 
      Kp.l_leg_hpy = 2200.0; 
      Kp.l_leg_kny = 3500.0; 
      Kp.l_leg_aky = 3300.0;
      Kp.l_leg_akx = 3300.0;
      Kp.neck_ay = 40.0; %%%%%
      Kp.back_bkz = 4500.0; % 
      Kp.back_bky = 6000.0; %
      Kp.back_bkx = 2350.0; %

      Kd.l_arm_usy = 38.0; %%%%%
      Kd.l_arm_shx = 75.0; %%%%%
      Kd.l_arm_ely = 30.0; %%%%%
      Kd.l_arm_elx = 60.0; %%%%%
      Kd.l_arm_uwy = 1.0; %%%%%
      Kd.l_arm_mwx = 10.0; %%%%%
      Kd.l_leg_hpz = 55.0; 
      Kd.l_leg_hpx = 95.0; 
      Kd.l_leg_hpy = 50.0; 
      Kd.l_leg_kny = 85.0; 
      Kd.l_leg_aky = 90.0;
      Kd.l_leg_akx = 90.0;  
      Kd.neck_ay = 4.0; %%%%%
      Kd.back_bkz = 50.0; %
      Kd.back_bky = 65.0; %
      Kd.back_bkx = 105.0; %
    else
      error('unknown mode given. valid modes are: default.');
    end

    % copy left gains to right side
    Kp.r_arm_usy = Kp.l_arm_usy;
    Kp.r_arm_shx = Kp.l_arm_shx;
    Kp.r_arm_ely = Kp.l_arm_ely;
    Kp.r_arm_elx = Kp.l_arm_elx;
    Kp.r_arm_uwy = Kp.l_arm_uwy;
    Kp.r_arm_mwx = Kp.l_arm_mwx;
    Kp.r_leg_hpz = Kp.l_leg_hpz;
    Kp.r_leg_hpx = Kp.l_leg_hpx;
    Kp.r_leg_hpy = Kp.l_leg_hpy;
    Kp.r_leg_kny = Kp.l_leg_kny;
    Kp.r_leg_aky = Kp.l_leg_aky;
    Kp.r_leg_akx = Kp.l_leg_akx;

    Kd.r_arm_usy = Kd.l_arm_usy;
    Kd.r_arm_shx = Kd.l_arm_shx;
    Kd.r_arm_ely = Kd.l_arm_ely;
    Kd.r_arm_elx = Kd.l_arm_elx;
    Kd.r_arm_uwy = Kd.l_arm_uwy;
    Kd.r_arm_mwx = Kd.l_arm_mwx;
    Kd.r_leg_hpz = Kd.l_leg_hpz;
    Kd.r_leg_hpx = Kd.l_leg_hpx;
    Kd.r_leg_hpy = Kd.l_leg_hpy;
    Kd.r_leg_kny = Kd.l_leg_kny;
    Kd.r_leg_aky = Kd.l_leg_aky;
    Kd.r_leg_akx = Kd.l_leg_akx;
  case 4
    if strcmpi(mode,'default')
      Kp.l_arm_shz = 625.0; %%%%%
      Kp.l_arm_shx = 3000.0; %%%%%
      Kp.l_arm_ely = 650.0; %%%%%
      Kp.l_arm_elx = 1600.0; %%%%%s
      Kp.l_arm_uwy = 25.0; %%%%%
      Kp.l_arm_mwx = 375.0; %%%%%
      Kp.l_leg_hpz = 800.0; 
      Kp.l_leg_hpx = 6000.0; 
      Kp.l_leg_hpy = 2200.0; 
      Kp.l_leg_kny = 3500.0; 
      Kp.l_leg_aky = 3300.0;
      Kp.l_leg_akx = 3300.0;
      Kp.neck_ay = 40.0; %%%%%
      Kp.back_bkz = 4500.0; % 
      Kp.back_bky = 6000.0; %
      Kp.back_bkx = 2350.0; %

      Kd.l_arm_shz = 38.0; %%%%%
      Kd.l_arm_shx = 75.0; %%%%%
      Kd.l_arm_ely = 30.0; %%%%%
      Kd.l_arm_elx = 60.0; %%%%%
      Kd.l_arm_uwy = 1.0; %%%%%
      Kd.l_arm_mwx = 10.0; %%%%%
      Kd.l_leg_hpz = 55.0; 
      Kd.l_leg_hpx = 95.0; 
      Kd.l_leg_hpy = 50.0; 
      Kd.l_leg_kny = 85.0; 
      Kd.l_leg_aky = 90.0;
      Kd.l_leg_akx = 90.0;  
      Kd.neck_ay = 4.0; %%%%%
      Kd.back_bkz = 50.0; %
      Kd.back_bky = 65.0; %
      Kd.back_bkx = 105.0; %
    else
      error('unknown mode given. valid modes are: default.');
    end

    % copy left gains to right side
    Kp.r_arm_shz = Kp.l_arm_shz;
    Kp.r_arm_shx = Kp.l_arm_shx;
    Kp.r_arm_ely = Kp.l_arm_ely;
    Kp.r_arm_elx = Kp.l_arm_elx;
    Kp.r_arm_uwy = Kp.l_arm_uwy;
    Kp.r_arm_mwx = Kp.l_arm_mwx;
    Kp.r_leg_hpz = Kp.l_leg_hpz;
    Kp.r_leg_hpx = Kp.l_leg_hpx;
    Kp.r_leg_hpy = Kp.l_leg_hpy;
    Kp.r_leg_kny = Kp.l_leg_kny;
    Kp.r_leg_aky = Kp.l_leg_aky;
    Kp.r_leg_akx = Kp.l_leg_akx;

    Kd.r_arm_shz = Kd.l_arm_shz;
    Kd.r_arm_shx = Kd.l_arm_shx;
    Kd.r_arm_ely = Kd.l_arm_ely;
    Kd.r_arm_elx = Kd.l_arm_elx;
    Kd.r_arm_uwy = Kd.l_arm_uwy;
    Kd.r_arm_mwx = Kd.l_arm_mwx;
    Kd.r_leg_hpz = Kd.l_leg_hpz;
    Kd.r_leg_hpx = Kd.l_leg_hpx;
    Kd.r_leg_hpy = Kd.l_leg_hpy;
    Kd.r_leg_kny = Kd.l_leg_kny;
    Kd.r_leg_aky = Kd.l_leg_aky;
    Kd.r_leg_akx = Kd.l_leg_akx;

end

Kp = diag(double(Kp));
Kd = diag(double(Kd));

end
