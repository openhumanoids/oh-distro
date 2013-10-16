function atlasGravityCompensation
%NOTEST

armstr = 'arm';

% load robot model
options.floating = true;
options.ignore_friction = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_stumps.urdf'),options);

options.floating = false;
r_fixed = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_stumps.urdf'),options);
arm_joints_act_fixed= ~cellfun(@isempty,strfind(r_fixed.getInputFrame.coordinates,armstr));

% setup frames
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosTorqueRef(r);

nq = getNumDOF(r);
arm_joints = find(~cellfun(@isempty,strfind(state_frame.coordinates(1:nq),armstr)));
arm_joints_act = find(~cellfun(@isempty,strfind(input_frame.coordinates,armstr)));

nu = getNumInputs(r);

joint_index_map = struct(); % maps joint names to indices
for i=1:nq
  joint_index_map.(state_frame.coordinates{i}) = i;
end

Fc_pos = Point(state_frame,0);
Fc_neg = Point(state_frame,0);

Fc_pos.r_arm_usy = 11;
Fc_pos.r_arm_shx = 12;
Fc_pos.r_arm_ely = 8.5;
Fc_pos.r_arm_elx = 9.0;
Fc_pos.r_arm_uwy = 8.0;
Fc_pos.r_arm_mwx = 6.5;

Fc_neg.r_arm_usy = 11;
Fc_neg.r_arm_shx = 12;
Fc_neg.r_arm_ely = 8.5;
Fc_neg.r_arm_elx = 9.0;
Fc_neg.r_arm_uwy = 8.0;
Fc_neg.r_arm_mwx = 5.5;

Fc_pos.l_arm_usy = 11;
Fc_pos.l_arm_shx = 13;
Fc_pos.l_arm_ely = 9;
Fc_pos.l_arm_elx = 9.5;
Fc_pos.l_arm_uwy = 8.0;
Fc_pos.l_arm_mwx = 6.5;

Fc_neg.l_arm_usy = 11;
Fc_neg.l_arm_shx = 13;
Fc_neg.l_arm_ely = 9;
Fc_neg.l_arm_elx = 9.5;
Fc_neg.l_arm_uwy = 8.0;
Fc_neg.l_arm_mwx = 5.5;

Fc_pos = double(Fc_pos);
Fc_neg = double(Fc_neg);

gains = getAtlasGains(input_frame); 

% zero out force gains to start --- move to nominal joint position
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
ref_frame.updateGains(gains);

% qdes = zeros(nq,1);
% qdes(joint_index_map.r_arm_shx) = 1.45; 
% qdes(joint_index_map.l_arm_shx) = -1.45; 
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
qdes = xstar(1:nq);

act_idx = getActuatedJoints(r);
atlasLinearMoveToPos(qdes,state_frame,ref_frame,act_idx,4);

gains2 = getAtlasGains(input_frame); 
gains.k_f_p(arm_joints_act) = gains2.k_f_p(arm_joints_act);
gains.ff_f_d(arm_joints_act) = gains2.ff_f_d(arm_joints_act);
gains.ff_qd(arm_joints_act) = gains2.ff_qd(arm_joints_act);
gains.ff_const(arm_joints_act) = gains2.ff_const(arm_joints_act);
% set joint position gains to 0
gains.k_q_p(arm_joints_act) = 0;
gains.k_q_i(arm_joints_act) = 0;
gains.k_qd_p(arm_joints_act) = 0;

ref_frame.updateGains(gains);
udes = zeros(nu,1);

% kalman filter params
H = [eye(nq) zeros(nq)];
R = 5e-4*eye(nq);
P = eye(2*nq);
x_est = zeros(2*nq,1);

toffset = -1;
tt=-1;

while 1
    [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
      tlast=0;
    end
    tt=t-toffset;
    dt = tt-tlast;
    
    F = [eye(nq) dt*eye(nq); zeros(nq) eye(nq)];
    Q = 0.3*[dt*eye(nq) zeros(nq); zeros(nq) eye(nq)];
    
    % compute filtered velocity
    jprior = F*x_est;
    Pprior = F*P*F' + Q;
    meas_resid = x(1:nq) - H*jprior;
    S = H*Pprior*H' + R;
    K = (P*H')/S;
    x_est = jprior + K*meas_resid;
    P = (eye(2*nq) - K*H)*Pprior;
    
    q = x_est(1:nq);
    qd = x_est(nq+(1:nq));
    
    Fv = 0.5;
    Fc_window = 0.175;
    
    tau_friction = zeros(34,1);
    for i=1:length(arm_joints)
      j=arm_joints(i);
      if qd(j)> 0
        tau_friction(j) = max(-1,min(1,qd(j)/Fc_window)) .* Fc_pos(j) + Fv*qd(j); 
      else
        tau_friction(j) = max(-1,min(1,qd(j)/Fc_window)) .* Fc_neg(j) + Fv*qd(j); 
      end
    end
    
    tf_act = tau_friction(act_idx);
    
    % do inverse dynamics on fixed base model
    nq_fixed = getNumDOF(r_fixed);
    [~,C,B] = manipulatorDynamics(r_fixed,q(6+(1:nq_fixed)),qd(6+(1:nq_fixed)));
    
    u = B\C;
    f_grav = u(arm_joints_act_fixed);
       
    % send torque command
    udes(arm_joints_act) = 0.5*tf_act(arm_joints_act) + f_grav;
    ref_frame.publish(t,[qdes(act_idx);udes],'ATLAS_COMMAND');
    tlast =tt;
  end
end
end
