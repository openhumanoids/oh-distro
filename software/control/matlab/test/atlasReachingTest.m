function atlasReachingTest
%NOTEST

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% hand goal positions (pelvis pos) ---ROTATIONS NOT SUPPORTED YET
rhand_goal = [0.33; -0.5; 0.3];
lhand_goal = [0.45; 0.15; 0.4];
neck_pitch = 0.6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
v = r.constructVisualizer;

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = AtlasPositionRef(r);

nq = getNumDOF(r);

load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
qstar = xstar(1:nq);

neck_idx = ~cellfun(@isempty,strfind(state_frame.coordinates(1:nq),'neck_ay'));
qstar(neck_idx) = neck_pitch;

disp('Moving to nominal pose (8 sec).');
act_idx = getActuatedJoints(r);
x=atlasLinearMoveToPos(qstar,state_frame,input_frame,act_idx,8);
q0=x(1:nq);
q0(4:6)=qstar(4:6); % temp: override rotations


kinsol = doKinematics(r,q0);
 
% generate manip plan
rhand_ind = findLinkInd(r,'r_hand');
lhand_ind = findLinkInd(r,'l_hand');

rhand_pos = forwardKin(r,kinsol,rhand_ind,[0;0;0],0);
lhand_pos = forwardKin(r,kinsol,lhand_ind,[0;0;0],0);

rfoot_ind = findLinkInd(r,'r_foot');
lfoot_ind = findLinkInd(r,'l_foot');
rfoot_pos = forwardKin(r,kinsol,rfoot_ind,[0;0;0],1);
lfoot_pos = forwardKin(r,kinsol,lfoot_ind,[0;0;0],1);

pelvis_ind = findLinkInd(r,'pelvis');
pelvis_pos = forwardKin(r,kinsol,pelvis_ind,[0;0;0],0);


cost = Point(r.getStateFrame,1);
cost.base_x = 10;
cost.base_y = 10;
cost.base_z = 10;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 10;
cost.back_bky = 50;
cost.back_bkx = 50;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:r.getNumDOF));
options.quastiStaticFlag = true;
options.q_nom = q0;%qstar;

T = 8;
ts = 0:0.025:T;

resp = 'retry';
while strcmp(resp,'retry')
  disp('Solving IK.');
  % time spacing of samples for IK

  rhand_traj = PPTrajectory(spline([0 T],[rhand_pos rhand_goal+pelvis_pos]));
  lhand_traj = PPTrajectory(spline([0 T],[lhand_pos lhand_goal+pelvis_pos]));
  for i=1:length(ts)
    t = ts(i);
    if (i>1)
       q(:,i) = inverseKin(r,q(:,i-1), ...
        rfoot_ind,[0;0;0],rfoot_pos, ...
        lfoot_ind,[0;0;0],lfoot_pos, ...
        rhand_ind,[0;0;0],rhand_traj.eval(t), ...
        lhand_ind,[0;0;0],lhand_traj.eval(t),options);
    else
      q = q0;
    end
  end

  qtraj = PPTrajectory(spline(ts,q));

  % draw plan in drake viewer
  xtraj = [qtraj; zeros(nq,1)];
  xtraj= xtraj.setOutputFrame(getOutputFrame(r));
  playback(v,xtraj,struct('slider',true));

  resp = input('Approve motion (y/n/retry/quit): ','s');
end

if any(strcmp(resp,{'y','yes'}))

  toffset = -1;
  tt=-1;
  while tt<T
    [x,t] = getNextMessage(state_frame,1);
    if ~isempty(x)
      if toffset==-1
        toffset=t;
      end
      tt=t-toffset;
      qdes = qtraj.eval(tt);
      input_frame.publish(t,qdes(act_idx),'ATLAS_COMMAND');
    end
  end

end
