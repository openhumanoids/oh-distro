function h = myplot(varargin)

p = varargin{1};
h = plot(p(:,1),p(:,2),varargin{2:end});
