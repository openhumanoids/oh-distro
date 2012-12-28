function [Kp,Kd] = getPDGains(r)

B = r.manip.model.B;
idx = B'*(1:r.getNumStates()/2)';

Kp = Point(CoordinateFrame('q_d',length(idx),'d',{r.getStateFrame.coordinates{idx}}));
Kd = Point(CoordinateFrame('q_d',length(idx),'d',{r.getStateFrame.coordinates{idx}}));

big = 1000;
med = 200;
sml = 20;

Kp.back_lbz = big;
Kp.back_mby = big;
Kp.back_ubx = big;
Kp.l_arm_elx = med;
Kp.l_arm_ely = med;
Kp.l_arm_mwx = sml;
Kp.l_arm_shx = big;
Kp.l_arm_usy = med;
Kp.l_arm_uwy = sml;
Kp.l_leg_kny = big;
Kp.l_leg_lax = med;
Kp.l_leg_lhy = big;
Kp.l_leg_mhx = big;
Kp.l_leg_uay = big;
Kp.l_leg_uhz = med;
Kp.neck_ay = sml;
Kp.r_arm_elx = med;
Kp.r_arm_ely = med;
Kp.r_arm_mwx = sml;
Kp.r_arm_shx = big;
Kp.r_arm_usy = med;
Kp.r_arm_uwy = sml;
Kp.r_leg_kny = big;
Kp.r_leg_lax = med;
Kp.r_leg_lhy = big;
Kp.r_leg_mhx = big;
Kp.r_leg_uay = big;
Kp.r_leg_uhz = med;

big = 10;
med = 2;
sml = 0.5;

Kd.back_lbz = big;
Kd.back_mby = big;
Kd.back_ubx = big;
Kd.l_arm_elx = med;
Kd.l_arm_ely = med;
Kd.l_arm_mwx = sml;
Kd.l_arm_shx = big;
Kd.l_arm_usy = med;
Kd.l_arm_uwy = sml;
Kd.l_leg_kny = big;
Kd.l_leg_lax = med;
Kd.l_leg_lhy = big;
Kd.l_leg_mhx = big;
Kd.l_leg_uay = big;
Kd.l_leg_uhz = med;
Kd.neck_ay = sml;
Kd.r_arm_elx = med;
Kd.r_arm_ely = med;
Kd.r_arm_mwx = sml;
Kd.r_arm_shx = big;
Kd.r_arm_usy = med;
Kd.r_arm_uwy = sml;
Kd.r_leg_kny = big;
Kd.r_leg_lax = med;
Kd.r_leg_lhy = big;
Kd.r_leg_mhx = big;
Kd.r_leg_uay = big;
Kd.r_leg_uhz = med;

Kp = diag(double(Kp));
Kd = diag(double(Kd));

end