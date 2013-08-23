function x=atlasLinearMoveToPos(qdes,state_frame,input_frame,act_idx,T)

% qdes desired configuration in state ordering

if nargin < 4
  T=5.0;
end

if isa(input_frame,'AtlasPositionRef')
  input_mode = 1;
elseif isa(input_frame,'AtlasPosTorqueRef')
  input_mode = 2;
elseif isa(input_frame,'AtlasPosVelTorqueRef')
  input_mode = 3;
else
  error('input frame must be of type AtlasPositionRef, AtlasPosTorqueRef, or AtlasPosVelTorqueRef')
end

toffset = -1;
tt=-1;
while tt<T
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
      q0 = x(act_idx);
      q_d_traj = PPTrajectory(foh([0,T],[q0,qdes(act_idx)]));
    end
    tt=t-toffset;
    q_d = q_d_traj.eval(tt);
    if input_mode==1
      input_frame.publish(t,q_d,'ATLAS_COMMAND');
    elseif input_mode==2
      input_frame.publish(t,[q_d;0*q_d],'ATLAS_COMMAND');
    elseif input_mode==3
      input_frame.publish(t,[q_d;0*q_d;0*q_d],'ATLAS_COMMAND');
    end
  end
end



