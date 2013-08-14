% close all;
clear all;

order = {'l_arm_mwx';
         'l_arm_uwy';
         'l_arm_elx';
         'l_arm_ely'; %Something iffy about this one
         'l_arm_shx';
         'l_arm_usy';
         'neck_ay';
         'back_bkx'; %Something iffy about this one
         'back_bky';
         'back_bkz'};

dof = order(8)
amplitude = .5;
tstep =4;
T=8;
tunePDPinnedGazebo(dof,amplitude,tstep,T);