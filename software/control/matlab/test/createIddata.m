function data = createIddata(plant,t_x,x_data,t_u,u_data,tspan,state_names,input_names)
% function data = createIddata(plant,t_x,x_data,t_u,u_data,tspan,state_names,input_names)
%   Construct an iddata object (for sys-id) from a data set
%   Uses the time series of the state data and assumes that this is sampled
%   at a constant rate. The control inputs are assumed to be a zero order
%   hold, and are matched to the state sequence under this assumption.
%
%   NOTEST
%
%   @param plant The plant object, used to compare coordinate frames. Can
%   be left empty if both state_names and input_names are not included
%   @param t_x Time vector related to state data
%   @param x_data State data, each column relates to one measurement
%   @param t_u Time vector related to input data
%   @param u_data Control input data
%   @param tspan Optional argument to only use a subset of the data
%   [tspan(1) tspan(2)]
%   @param state_names Optional argument. Cell arrray of state names, as
%   from plant.getStateFrame.coordinates. If specified, only uses a subset
%   of the state data
%   @param input_names Optional argument. Cell arrray of input names, as
%   from plant.getInputFrame.coordinates. If specified, only uses a subset
%   of the input data.
%   @return data The iddata
dt = diff(t_x);
if max(dt) - min(dt) > 1e-8
  error('Sampling rate must be constant')
end
dt = dt(1);

if nargin > 5
  sizecheck(tspan,[1 2]);
  x_ind = find(t_x <= tspan(2) & t_x >= tspan(1));
  t_data = t_x(x_ind);
  x_interp = x_data(:,x_ind);
else
  x_interp = x_data;
  t_data = t_x;
end

if nargin > 6
  state_ind = zeros(length(state_names),1);
  for i=1:length(state_names)
    state_ind(i) = find(strcmp(plant.getStateFrame.coordinates,state_names{i}));
  end
else
  state_ind = (1:size(x_data,1))';
end

if nargin > 7
  input_ind = zeros(length(input_names),1);
  for i=1:length(input_names)
    input_ind(i) = find(strcmp(plant.getInputFrame.coordinates,input_names{i}));
  end
else
  input_ind = (1:size(u_data,1))';
end

u_interp = interpZOH(t_u, u_data, t_data);
data = iddata(x_interp(state_ind,:)',u_interp(input_ind,:)',dt);

  function yi = interpZOH(x,y,xi)
    inds = arrayfun(@findinds, xi);
    yi = y(:,inds);
    
    function ind = findinds(val)
      ind = find(x <= val,1,'last');
      if isempty(ind)
        ind = 1;
      end
    end
  end
end
