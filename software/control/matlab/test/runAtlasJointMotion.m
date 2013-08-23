function runAtlasJointMotion(joint_name,config_ids,f0,fT,amp,T,zero_crossing,r,review_motion)
%NOTEST

% function for producing joint motions in position control mode to generate
% data for model fitting.

assert(length(f0)==length(fT));
assert(length(f0)==length(amp));
assert(length(f0)==length(T));

if nargin<8 
  % load robot model
  options.floating = true;
  options.dt = 0.002;
  r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
end

if nargin<9
  review_motion=true;
end

if review_motion
  v = r.constructVisualizer;
end

% setup frames
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
ref_frame = AtlasPosTorqueRef(r);

nu = getNumInputs(r);
nq = getNumDOF(r);

joint_index = find(strcmp(joint_name,r.getStateFrame.coordinates));
[jl_min,jl_max]=getJointLimits(r);

for config_id=config_ids
  for i=1:length(f0)
    fprintf([joint_name ': config #%d, f0=%2.2f, fT=%2.2f, amp=%2.2f, T=%2.2f\n'],config_id,f0(i),fT(i),amp(i),T(i));

    qdes = getAtlasJointMotionConfig(r,joint_name,config_id);
    qtraj = getAtlasChirpTraj(joint_index,qdes,f0(i),fT(i),amp(i),T(i),zero_crossing,jl_min,jl_max);

    if review_motion
      % draw plan in drake viewer
      xtraj = [qtraj; zeros(nq,1)];
      xtraj= xtraj.setOutputFrame(getOutputFrame(r));
      playback(v,xtraj);
%       playback(v,xtraj,struct('slider',true));
      resp = input('Approve motion (y/n): ','s');
      if ~strcmp(resp,{'y','yes'})
        continue;
      end
    end

    % move to starting position for motion
    qdes = qtraj.eval(0);
    act_idx = getActuatedJoints(r);
    atlasLinearMoveToPos(qdes,state_frame,ref_frame,act_idx,4.0);
    
    tf = qtraj.tspan(end);
    udes = zeros(nu,1);
    toffset = -1;
    tt=-1;
    while tt<tf
      [x,t] = getNextMessage(state_frame,1);
      if ~isempty(x)
        if toffset==-1
          toffset=t;
        end
        tt=t-toffset;
        qdes = qtraj.eval(tt);
        ref_frame.publish(t,[qdes;udes],'ATLAS_COMMAND');
      end
    end
  end
end