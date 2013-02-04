function [Kp,Kd] = getPDGains(r)

%NOTEST

idx = r.getActuatedJoints();

fr=CoordinateFrame('q_d',length(idx),'d',{r.getStateFrame.coordinates{idx}});
Kp = Point(fr);
Kd = Point(fr);

Kp.l_leg_uhz = 800.0; 
Kp.l_leg_mhx = 6000.0; 
Kp.l_leg_lhy = 2200.0; 
Kp.l_leg_kny = 3500.0; 
Kp.l_leg_uay = 3300.0;
Kp.l_leg_lax = 3300.0;
Kp.r_leg_uhz = Kp.l_leg_uhz;
Kp.r_leg_mhx = Kp.l_leg_mhx;
Kp.r_leg_lhy = Kp.l_leg_lhy;
Kp.r_leg_kny = Kp.l_leg_kny;
Kp.r_leg_uay = Kp.l_leg_uay;
Kp.r_leg_lax = Kp.l_leg_lax;
Kp.l_arm_usy = 625.0; %%%%%
Kp.l_arm_shx = 3000.0; %%%%%
Kp.l_arm_ely = 650.0; %%%%%
Kp.l_arm_elx = 1600.0; %%%%%
Kp.l_arm_uwy = 25.0; %%%%%
Kp.l_arm_mwx = 375.0; %%%%%
Kp.r_arm_usy = Kp.l_arm_usy;
Kp.r_arm_shx = Kp.l_arm_shx;
Kp.r_arm_ely = Kp.l_arm_ely;
Kp.r_arm_elx = Kp.l_arm_elx;
Kp.r_arm_uwy = Kp.l_arm_uwy;
Kp.r_arm_mwx = Kp.l_arm_mwx;
Kp.neck_ay = 40.0; %%%%%
Kp.back_lbz = 4500.0; % 
Kp.back_mby = 6000.0; %
Kp.back_ubx = 2350.0; %

Kd.l_leg_uhz = 55.0; 
Kd.l_leg_mhx = 95.0; 
Kd.l_leg_lhy = 50.0; 
Kd.l_leg_kny = 85.0; 
Kd.l_leg_uay = 90.0;
Kd.l_leg_lax = 90.0;
Kd.r_leg_uhz = Kd.l_leg_uhz;
Kd.r_leg_mhx = Kd.l_leg_mhx;
Kd.r_leg_lhy = Kd.l_leg_lhy;
Kd.r_leg_kny = Kd.l_leg_kny;
Kd.r_leg_uay = Kd.l_leg_uay;
Kd.r_leg_lax = Kd.l_leg_lax;
Kd.l_arm_usy = 38.0; %%%%%
Kd.l_arm_shx = 75.0; %%%%%
Kd.l_arm_ely = 30.0; %%%%%
Kd.l_arm_elx = 60.0; %%%%%
Kd.l_arm_uwy = 1.0; %%%%%
Kd.l_arm_mwx = 10.0; %%%%%
Kd.r_arm_usy = Kd.l_arm_usy;
Kd.r_arm_shx = Kd.l_arm_shx;
Kd.r_arm_ely = Kd.l_arm_ely;
Kd.r_arm_elx = Kd.l_arm_elx;
Kd.r_arm_uwy = Kd.l_arm_uwy;
Kd.r_arm_mwx = Kd.l_arm_mwx;
Kd.neck_ay = 4.0; %%%%%
Kd.back_lbz = 50.0; %
Kd.back_mby = 65.0; %
Kd.back_ubx = 105.0; %

Kp = diag(double(Kp));
Kd = diag(double(Kd));

end