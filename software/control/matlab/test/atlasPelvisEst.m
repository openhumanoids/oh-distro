function atlasPelvisEst
% load robot model
r = Atlas();

% setup frames
state_frame = AtlasState(r);
state_frame.subscribe('EST_ROBOT_STATE');
nq = r.getNumDOF();

process_noise = 0.01*ones(nq,1);
observation_noise = 5e-4*ones(nq,1);
kf = FirstOrderKalmanFilter(process_noise,observation_noise);
kf_state = kf.getInitialState;

alpha_lin = 0.1;
alpha_ang = 0.1;
pelvis_lin = zeros(3,1);
pelvis_ang = zeros(3,1);
while 1
  [x,t] = getNextMessage(state_frame,10);
  if ~isempty(x)
    x_raw=x;
    % get estimated state
    kf_state = kf.update(t,kf_state,x(1:nq));
    x_kf = kf.output(t,kf_state,x(1:nq));

    x_lowpass = x_raw;
    
    pelvis_lin = (1-alpha_lin)*pelvis_lin + alpha_lin*x_raw(nq+(1:3));
    pelvis_ang = (1-alpha_ang)*pelvis_ang + alpha_ang*x_raw(nq+(4:6));
    x_lowpass(nq+(1:3)) = pelvis_lin;
    x_lowpass(nq+(4:6)) = pelvis_ang;
    
    state_frame.publish(t,x_raw,'EST_ROBOT_STATE_ECHO');
    state_frame.publish(t,x_kf,'EST_ROBOT_STATE_KF');
    state_frame.publish(t,x_lowpass,'EST_ROBOT_STATE_LP');
  end
end
