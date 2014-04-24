% load robot model
r = Atlas();
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));

% setup frames
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');

toffset=-1;
tt=-1;
X = [];
while tt<5
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
    end
    tt=t-toffset;

    X = [X x];      
  end
end

N=size(X,2);
x_bar=mean(X,2);
Q = 1/(N-1) * (X-x_bar*ones(1,N)) *(X-x_bar*ones(1,N))';
