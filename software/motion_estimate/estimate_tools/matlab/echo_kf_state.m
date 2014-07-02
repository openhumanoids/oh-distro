r = Atlas();

state_frame = r.getStateFrame();
state_frame.subscribe('EST_ROBOT_STATE_NO_FILTER');
input_frame = AtlasPosVelTorqueRef(r);

nq = getNumDOF(r);
process_noise = 0.01*ones(nq,1);
observation_noise = 5e-4*ones(nq,1);
kf = FirstOrderKalmanFilter(process_noise,observation_noise);
kf_state = kf.getInitialState;

act_idx = getActuatedJoints(r);


counter = 0;

while 1
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    % get estimated state
    kf_state = kf.update(t,kf_state,x(1:nq));
    x_kf = kf.output(t,kf_state,x(1:nq));
    
    kf_q = x_kf(act_idx);
    kf_qd = x_kf(nq+act_idx);
    
    input_frame.publish(t,[kf_q; kf_qd; 0*kf_q],'EST_ROBOT_STATE_KF');
    
    
    counter = counter + 1;
    
    
    
  end
end