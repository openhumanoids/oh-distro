function qtraj = getAtlasChirpTraj(joint_index,qdes,f0,fT,amp,T,zero_crossing,jl_min,jl_max)
%NOTEST

% function for producing joint motions in position control mode to generate
% data for model fitting

% note: the sign of amp determines the direction of motion when
% zero_crossing==false

if nargin < 7
  zero_crossing = false;
end

n=400;
ts = linspace(0,T,n);
freq = linspace(f0,fT,n);
qtraj = ConstantTrajectory(qdes);

if zero_crossing
  jtraj = qdes(joint_index) + amp*sin(ts.*freq*2*pi);
else
  jtraj = qdes(joint_index) + 0.5*amp-0.5*amp*cos(ts.*freq*2*pi);
end

qtraj(joint_index) = PPTrajectory(foh(ts,jtraj));

if nargin > 7
  if any(qdes < jl_min) || any(jtraj < jl_min(joint_index))
    warning('getAtlasJointMotion: trajectory exceeds minimum joint limits');
    keyboard;
  end
end

if nargin > 8
  if any(qdes > jl_max) || any(jtraj > jl_max(joint_index))
    warning('getAtlasJointMotion: trajectory exceeds maximum joint limits');
    keyboard;
  end
end


end