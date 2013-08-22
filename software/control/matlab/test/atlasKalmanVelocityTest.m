function atlasKalmanVelocityTest
%NOTEST

% Applies a kalman filter to the estimated velocities and pushes it out as
% a commanded velocity (for now, just to easily visualize in signal-scope)

% load robot model
options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
nu = getNumInputs(r);

% setup frames
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosVelTorqueRef(r);

gains = getAtlasGains(input_frame); 
gains.k_q_p = zeros(nu,1);
gains.k_q_i = zeros(nu,1);
gains.k_f_p = zeros(nu,1);
gains.k_qd_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
gains.ff_qd_d = zeros(nu,1);
gains.ff_const = zeros(nu,1);
ref_frame.updateGains(gains);

act_idx = getActuatedJoints(r);
q_des = zeros(nu,1);
qd_des = zeros(nu,1);
u_des = zeros(nu,1);

nq = getNumDOF(r);
% kalman filter params
H = [eye(nq) zeros(nq)];
R = 5e-4*eye(nq);
P = eye(2*nq);


x_est = zeros(2*nq,1);

toffset = -1;
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
    
    % set q_des, qd_des just for plotting (gains are zero)
    q_des = x_est(act_idx);
    qd_des = x_est(nq+act_idx);
        
    ref_frame.publish(t,[q_des;qd_des;u_des],'ATLAS_COMMAND');
    tlast =tt;
  end
end
end
