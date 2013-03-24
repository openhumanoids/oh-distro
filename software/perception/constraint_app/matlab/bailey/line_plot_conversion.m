function p= line_plot_conversion (lne)
%function p= line_plot_conversion (lne)
%
% INPUT: list of lines [x1;y1;x2;y2]
% OUTPUT: list of points [x;y]
%
% Convert a list of lines so that they will be plotted as a set of
% unconnected lines but only require a single handle to do so. This
% is performed by converting the lines to a set of points, where a
% NaN point is inserted between every point-pair:
%
%   l= [x1a x1b x1c;
%       y1a y1b y1c;
%       x2a x2b x2c;
%       y2a y2b y2c];
%
%   becomes
%
%   p= [x1a x2a NaN x1b x2b NaN x1c x2c;
%       y1a y2a NaN y1b y2b NaN y1c y2c];
%
% Tim Bailey 2002. Thanks to Jose Guivant for this 'discrete line segments' 
% plotting technique.

len= size(lne,2)*3 - 1;
p= zeros(2, len);

p(:,1:3:end)= lne(1:2,:);
p(:,2:3:end)= lne(3:4,:);
p(:,3:3:end)= NaN;
