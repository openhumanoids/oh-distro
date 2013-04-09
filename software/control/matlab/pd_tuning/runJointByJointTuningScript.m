close all;
clear all;

dofnames ={'l_arm_mwx'
'l_arm_uwy'
'l_arm_elx'
'l_arm_ely'
'l_arm_shx'
'l_arm_usy'
'neck_ay'
'l_leg_lax'
'l_leg_uay'
'l_leg_kny'
'l_leg_lhy'
'l_leg_mhx'
'l_leg_uhz'
'back_ubx'
'back_mby'
'back_lbz'};

ind=16;

amplitude=1.5/6;
tstep=3;
tstep=0.01;
T=12;
tunePDPinnedGazebo(dofnames{ind},amplitude,tstep,T);