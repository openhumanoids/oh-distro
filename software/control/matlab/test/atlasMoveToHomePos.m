function atlasMoveToHomePos
%NOTEST

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
v = r.constructVisualizer;
v.display_dt = 0.01;

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = AtlasPositionRef(r);

nq = getNumDOF(r);

load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
qstar = xstar(1:nq);

act_idx = getActuatedJoints(r);

disp('Moving to nominal pose (10 sec).');

% move to desired initial pose
movetime = 10.0;
toffset = -1;
tt=-1;
while tt<movetime
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    q0=x(1:nq);
    if toffset==-1
      toffset=t;
      qtraj = PPTrajectory(foh([0,movetime],[q0,qstar]));
    end
    tt=t-toffset;
    qdes = qtraj.eval(tt);

    input_frame.publish(t,qdes(act_idx),'ATLAS_COMMAND');
  end
end

