function h = myplot3(varargin)

p = varargin{1};
h = plot3(p(:,1),p(:,2),p(:,3),varargin{2:end});
