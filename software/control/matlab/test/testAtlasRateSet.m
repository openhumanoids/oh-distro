function testAtlasRateSet
%NOTEST

% load robot model
r = Atlas();

% setup frames
state_frame = r.getStateFrame;
state_frame.subscribe('EST_ROBOT_STATE');

t_prev = 0;
alpha = 0.8;
rate_filt = 0;
while true
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    dt = t-t_prev;
    rate_filt = alpha*rate_filt + (1-alpha)*(1/dt);
    fprintf('rate: %4.2f Hz\n',rate_filt);
    t_prev = t;
  end
end

end
