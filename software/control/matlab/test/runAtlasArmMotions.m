function runAtlasArmMotions(review_motions)
%NOTEST

if nargin<1
  review_motions = false;
end

left_arm = true;
right_arm = false;

% load robot model
options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

f0 = 0.025;
fT = 0.5;
T = 40; 

if left_arm
  amp=pi/5;
  runAtlasJointMotion('l_arm_usy',[1 2 3],f0,fT,amp,T,true,r,review_motions);

  amp=pi/4;
  runAtlasJointMotion('l_arm_shx',[1 2],f0,fT,amp,T,false,r,review_motions);
  amp=pi/5;
  runAtlasJointMotion('l_arm_shx',3,f0,fT,amp,T,true,r,review_motions);

  amp=pi/5;
  runAtlasJointMotion('l_arm_ely',1,f0,fT,amp,T,true,r,review_motions);
  amp=pi/4;
  runAtlasJointMotion('l_arm_ely',[2 3],f0,fT,amp,T,true,r,review_motions);

  amp=pi/4;
  runAtlasJointMotion('l_arm_elx',1,f0,fT,amp,T,false,r,review_motions);
  amp=pi/5;
  runAtlasJointMotion('l_arm_elx',[2 3],f0,fT,amp,T,true,r,review_motions);

  amp=pi/4;
  runAtlasJointMotion('l_arm_uwy',[1 2 3],f0,fT,amp,T,false,r,review_motions);

  amp=pi/8;
  runAtlasJointMotion('l_arm_mwx',[1 2 3],f0,fT,amp,T,true,r,review_motions);
end

if right_arm
  amp=pi/5;
  runAtlasJointMotion('r_arm_usy',[1 2 3],f0,fT,amp,T,true,r,review_motions);

  amp=-pi/4;
  runAtlasJointMotion('r_arm_shx',[1 2],f0,fT,amp,T,false,r,review_motions);
  amp=pi/5;
  runAtlasJointMotion('r_arm_shx',3,f0,fT,amp,T,true,r,review_motions);

  amp=pi/5;
  runAtlasJointMotion('r_arm_ely',1,f0,fT,amp,T,true,r,review_motions);
  amp=pi/4;
  runAtlasJointMotion('r_arm_ely',[2 3],f0,fT,amp,T,true,r,review_motions);

  amp=-pi/4;
  runAtlasJointMotion('r_arm_elx',1,f0,fT,amp,T,false,r,review_motions);
  amp=pi/5;
  runAtlasJointMotion('r_arm_elx',[2 3],f0,fT,amp,T,true,r,review_motions);

  amp=pi/4;
  runAtlasJointMotion('r_arm_uwy',[1 2 3],f0,fT,amp,T,false,r,review_motions);

  amp=pi/8;
  runAtlasJointMotion('r_arm_mwx',[1 2 3],f0,fT,amp,T,true,r,review_motions);
end
