function qdes=getAtlasJointMotionConfig(r,joint_name,config_id)
%NOTEST

% function for producing joint motions in position control mode to generate
% data for model fitting

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
  switch config_id
    case 1
      qdes(joint_index_map.l_arm_shx) = -1.45;
      qdes(joint_index_map.l_arm_ely) = pi/2;
    case 2
      qdes(joint_index_map.l_arm_shx) = -1.45;
      qdes(joint_index_map.l_arm_elx) = pi/2;
      qdes(joint_index_map.l_arm_ely) = pi/2;
    case 3
      qdes(joint_index_map.l_arm_elx) = pi/2;
      qdes(joint_index_map.l_arm_ely) = pi/2;
    otherwise
      error('unknown config_id');
  end
  
elseif strcmp(joint_name,'l_arm_elx') || strcmp(joint_name,'l_arm_uwy') || strcmp(joint_name,'l_arm_mwx')
  qdes(joint_index_map.r_arm_shx) = 1.45;
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
  switch config_id
    case 1
      qdes(joint_index_map.r_arm_shx) = 1.45;
      qdes(joint_index_map.r_arm_ely) = pi/2;
    case 2
      qdes(joint_index_map.r_arm_shx) = 1.45;
      qdes(joint_index_map.r_arm_elx) = -pi/2;
      qdes(joint_index_map.r_arm_ely) = pi/2;
    case 3
      qdes(joint_index_map.r_arm_elx) = -pi/2;
      qdes(joint_index_map.r_arm_ely) = pi/2;
    otherwise
      error('unknown config_id');
  end
  
elseif strcmp(joint_name,'r_arm_elx') || strcmp(joint_name,'r_arm_uwy') || strcmp(joint_name,'r_arm_mwx')
  qdes(joint_index_map.l_arm_shx) = -1.45;
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
  
else
  error ('that joint isnt supported yet');
end
  
