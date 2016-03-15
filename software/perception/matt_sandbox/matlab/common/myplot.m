function h = myplot(data,varargin)

if (size(data,2)>2)
    h = myplot3(data, varargin);
else
    h = plot(data(:,1),data(:,2),varargin{:});
end