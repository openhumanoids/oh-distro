function [X,T] = recordRobotMotion(duration)
  S = warning();
  warning('OFF','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  r = Atlas();
  warning(S);
  nq = r.getNumDOF();
  t0 = [];
  t = 0;

  state_frame = getStateFrame(r);
  state_frame.subscribe('EST_ROBOT_STATE');
  display('Listening ...');
  X = zeros(2*nq,0);
  T = zeros(1,0);
  while isempty(t0) || t < t0 + duration
    [x,t] = getNextMessage(state_frame,10);
    if (~isempty(x))
      if isempty(t0)
        t0 = t;
      elseif t < t0
        t0 = t;
        X = zeros(2*nq,0);
        T = zeros(1,0);
      end
      fprintf('delta t = %5.2f\n',t-t0);
      X(:,end+1) = x;
      T(:,end+1) = t-t0;
    end
    pause(1e-3)
  end
end
