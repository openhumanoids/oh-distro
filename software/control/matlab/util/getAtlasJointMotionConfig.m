function [qdes,motion_direction]=getAtlasJointMotionConfig(r,joint_name,config_id)
%NOTEST

% function for producing joint motions in position control mode to generate
% data for gain tuning and model fitting

nq = getNumDOF(r);

% setup frames
state_frame = getStateFrame(r);

joint_index_map = struct(); % maps joint names to indices
for i=1:nq
  joint_index_map.(state_frame.coordinates{i}) = i;
end
if ~isfield(joint_index_map,joint_name)
  error ('unknown joint name');
end

if nargin < 3
  config_id = 1;
end

motion_direction = 1; % default move in positive direction
qdes = zeros(nq,1);
if strcmp(joint_name,'l_arm_usy')
  qdes(joint_index_map.r_arm_shx) = 1.45;
  switch config_id
    case 1
      qdes(joint_index_map.l_arm_shx) = -1.4;
    case 2
      qdes(joint_index_map.l_arm_shx) = -pi/4;
      qdes(joint_index_map.l_arm_elx) = 1.3;
    case 3
      qdes(joint_index_map.l_arm_elx) = pi/2;
      qdes(joint_index_map.l_arm_ely) = pi/2;
    otherwise
      error('unknown config_id');
  end
  
elseif strcmp(joint_name,'l_arm_shx')
  motion_direction = -1;
  qdes(joint_index_map.r_arm_shx) = 1.45;
  switch config_id
    case 1
      qdes(joint_index_map.l_arm_shx) = -1.45;
    case 2
      qdes(joint_index_map.l_arm_shx) = -pi/3;
      qdes(joint_index_map.l_arm_elx) = pi/2;
    case 3
      qdes(joint_index_map.l_arm_elx) = pi/2;
      qdes(joint_index_map.l_arm_ely) = pi/2;
    otherwise
      error('unknown config_id');
  end
  
elseif strcmp(joint_name,'l_arm_ely')
  qdes(joint_index_map.r_arm_shx) = 1.45;
  qdes(joint_index_map.l_arm_ely) = pi/2;
  switch config_id
    case 1
      qdes(joint_index_map.l_arm_shx) = -1.45;
    case 2
      qdes(joint_index_map.l_arm_shx) = -1.45;
      qdes(joint_index_map.l_arm_elx) = pi/2;
    case 3
      qdes(joint_index_map.l_arm_elx) = pi/2;
    otherwise
      error('unknown config_id');
  end
  
elseif any(strcmp(joint_name,{'l_arm_elx','l_arm_uwy','l_arm_mwx'}))
  qdes(joint_index_map.r_arm_shx) = 1.45;
  qdes(joint_index_map.l_arm_uwy) = pi/2;
  switch config_id
    case 1
      qdes(joint_index_map.l_arm_shx) = -1.45;
    case 2
      qdes(joint_index_map.l_arm_elx) = pi/2;
    case 3
      qdes(joint_index_map.l_arm_elx) = pi/2;
      qdes(joint_index_map.l_arm_ely) = pi/2;
    otherwise
      error('unknown config_id');
  end
  
elseif strcmp(joint_name,'r_arm_usy')
  qdes(joint_index_map.l_arm_shx) = -1.45;
  switch config_id
    case 1
      qdes(joint_index_map.r_arm_shx) = 1.4;
    case 2
      qdes(joint_index_map.r_arm_shx) = pi/4;
      qdes(joint_index_map.r_arm_elx) = -1.3;
    case 3
      qdes(joint_index_map.r_arm_elx) = -pi/2;
      qdes(joint_index_map.r_arm_ely) = pi/2;
    otherwise
      error('unknown config_id');
  end
  
elseif strcmp(joint_name,'r_arm_shx')
  qdes(joint_index_map.l_arm_shx) = -1.45;
  switch config_id
    case 1
      qdes(joint_index_map.r_arm_shx) = 1.45;
    case 2
      qdes(joint_index_map.r_arm_shx) = pi/3;
      qdes(joint_index_map.r_arm_elx) = -pi/2;
    case 3
      qdes(joint_index_map.r_arm_elx) = -pi/2;
      qdes(joint_index_map.r_arm_ely) = pi/2;
    otherwise
      error('unknown config_id');
  end
  
elseif strcmp(joint_name,'r_arm_ely')
  qdes(joint_index_map.l_arm_shx) = -1.45;
  qdes(joint_index_map.r_arm_ely) = pi/2;
  switch config_id
    case 1
      qdes(joint_index_map.r_arm_shx) = 1.45;
    case 2
      qdes(joint_index_map.r_arm_shx) = 1.45;
      qdes(joint_index_map.r_arm_elx) = -pi/2;
    case 3
      qdes(joint_index_map.r_arm_elx) = -pi/2;
    otherwise
      error('unknown config_id');
  end
  
elseif any(strcmp(joint_name,{'r_arm_elx','r_arm_uwy','r_arm_mwx'}))
  qdes(joint_index_map.l_arm_shx) = -1.45;
  qdes(joint_index_map.r_arm_uwy) = pi/2;
  switch config_id
    case 1
      qdes(joint_index_map.r_arm_shx) = 1.45;
    case 2
      qdes(joint_index_map.r_arm_elx) = -pi/2;
    case 3
      qdes(joint_index_map.r_arm_elx) = -pi/2;
      qdes(joint_index_map.r_arm_ely) = pi/2;
    otherwise
      error('unknown config_id');
  end

elseif strcmp(joint_name,'l_leg_hpy') 
  qdes(joint_index_map.r_arm_shx) = 1.25;
  qdes(joint_index_map.l_arm_shx) = -1.25;
  qdes(joint_index_map.r_leg_hpx) = -0.25;
  
elseif strcmp(joint_name,'r_leg_hpy') 
  qdes(joint_index_map.r_arm_shx) = 1.25;
  qdes(joint_index_map.l_arm_shx) = -1.25;
  qdes(joint_index_map.l_leg_hpx) = 0.25;

elseif strcmp(joint_name,'l_leg_hpz') 
  qdes(joint_index_map.r_arm_shx) = 1.25;
  qdes(joint_index_map.l_arm_shx) = -1.25;
  switch config_id
    case 1
    case 2
      qdes(joint_index_map.r_leg_hpx) = -0.4;
      qdes(joint_index_map.l_leg_kny) = 1.57;
    otherwise
      error('unknown config_id');
  end

elseif strcmp(joint_name,'r_leg_hpz') 
  qdes(joint_index_map.r_arm_shx) = 1.25;
  qdes(joint_index_map.l_arm_shx) = -1.25;
  switch config_id
    case 1
    case 2
      qdes(joint_index_map.l_leg_hpx) = 0.4;
      qdes(joint_index_map.r_leg_kny) = 1.57;
    otherwise
      error('unknown config_id');
  end
  
elseif strcmp(joint_name,'l_leg_hpx') 
  qdes(joint_index_map.r_arm_shx) = 0.7;
  qdes(joint_index_map.l_arm_shx) = -0.7;
  qdes(joint_index_map.r_leg_hpx) = -0.5;

elseif strcmp(joint_name,'r_leg_hpx') 
  qdes(joint_index_map.r_arm_shx) = 0.7;
  qdes(joint_index_map.l_arm_shx) = -0.7;
  qdes(joint_index_map.l_leg_hpx) = 0.5;
  motion_direction = -1;

elseif strcmp(joint_name,'l_leg_kny') 
  qdes(joint_index_map.r_arm_shx) = 1.25;
  qdes(joint_index_map.l_arm_shx) = -1.25;
  qdes(joint_index_map.l_leg_hpy) = -pi/2;
  qdes(joint_index_map.l_leg_kny) = pi/2;  

elseif strcmp(joint_name,'r_leg_kny') 
  qdes(joint_index_map.r_arm_shx) = 1.25;
  qdes(joint_index_map.l_arm_shx) = -1.25;
  qdes(joint_index_map.r_leg_hpy) = -pi/2;
  qdes(joint_index_map.r_leg_kny) = pi/2;  
  
else
  error ('that joint isnt supported yet');
end
  
