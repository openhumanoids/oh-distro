function atlasMoveToHomePos(duration)
%NOTEST

if nargin < 1
  duration=8; %sec  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
neck_pitch = 0.6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
%v = r.constructVisualizer;
%v.display_dt = 0.01;

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = AtlasPositionRef(r);

nq = getNumDOF(r);

load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));

qstar = xstar(1:nq);

neck_idx = ~cellfun(@isempty,strfind(state_frame.coordinates(1:nq),'neck_ay'));
qstar(neck_idx) = neck_pitch;

act_idx = getActuatedJoints(r);

disp(['Moving to nominal pose over ' num2str(duration) 'sec.']);

% move to desired initial pose
toffset = -1;
tt=-1;
while tt<duration
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    q0=x(1:nq);
    if toffset==-1
      toffset=t;
      qtraj = PPTrajectory(foh([0,duration],[q0,qstar]));
    end
    tt=t-toffset;
    qdes = qtraj.eval(tt);

    input_frame.publish(t,qdes(act_idx),'ATLAS_COMMAND');
  end
end

