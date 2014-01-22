function atlasGravityCompensation
%NOTEST

jointstr = 'arm';

% load robot model
options.floating = true;
options.ignore_friction = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_stumps.urdf'),options);

% load fixed-base model
options.floating = false;
options.ignore_friction = false;
r_fixed = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_stumps.urdf'),options);
joints_act_fixed = ~cellfun(@isempty,strfind(r_fixed.getInputFrame.coordinates,jointstr));

% setup frames
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosTorqueRef(r);

nq = getNumDOF(r);
nu = getNumInputs(r);

joints = find(~cellfun(@isempty,strfind(state_frame.coordinates(1:nq),jointstr)));
joints_act = find(~cellfun(@isempty,strfind(input_frame.coordinates,jointstr)));

joint_index_map = struct(); % maps joint names to indices
for i=1:nq
  joint_index_map.(state_frame.coordinates{i}) = i;
end

gains = getAtlasGains(input_frame); 
% zero out force gains to start --- move to nominal joint position
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
ref_frame.updateGains(gains);

% load in nominal posture
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
qdes = xstar(1:nq);

act_idx = getActuatedJoints(r);
atlasLinearMoveToPos(qdes,state_frame,ref_frame,act_idx,4);

% copy force control gains back in
gains2 = getAtlasGains(input_frame); 
gains.k_f_p(joints_act) = gains2.k_f_p(joints_act);
gains.ff_f_d(joints_act) = gains2.ff_f_d(joints_act);
gains.ff_qd(joints_act) = gains2.ff_qd(joints_act);
gains.ff_const(joints_act) = gains2.ff_const(joints_act);
% set joint position gains to 0
gains.k_q_p(joints_act) = 0;
gains.k_q_i(joints_act) = 0;
gains.k_qd_p(joints_act) = 0;

ref_frame.updateGains(gains);
udes = zeros(nu,1);

toffset = -1;
tt=-1;
while 1
    [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
    end
    tt=t-toffset;
    
    q = x(1:nq);
    qd = x(nq+(1:nq));
    
    % do inverse dynamics on fixed base model
    nq_fixed = getNumDOF(r_fixed);
    [~,C,B] = manipulatorDynamics(r_fixed,q(6+(1:nq_fixed)),qd(6+(1:nq_fixed)));    
    u = B\C;
       
    % send torque command
    udes(joints_act) = u(joints_act_fixed);
    ref_frame.publish(t,[qdes(act_idx);udes],'ATLAS_COMMAND');
    tlast =tt;
  end
end
end
