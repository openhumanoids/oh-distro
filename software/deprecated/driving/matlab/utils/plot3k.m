% Plot values in the color vector c at the locations in L = (x,y,z)
%    g = plot3k(L,c,marker,nlabels)
% Input
%    L        x,y,z location vector,                   n x 3
%    c        color vector,              default = z,  n x 1
%    marker   marker character,          default = '.'
%    nlabels  number of colorbar labels, default = 10
% Output
%    g        figure handle for the plot
%
% Generates a 3D point plot of L=(x,y,z) using the values in the vector c to
% determine the color of each point.  If c is empty, then z (column 3 of L) is
% used to color the plot.  The data points are sorted so that plot3 is only
% called once for each group of points that map to the same color.  A handle
% to the figure is returned.
%
% Example
%    [x,y,z] = peaks(101);
%    plot3k([x(:) y(:) z(:)],[],[],12);
%    - produces a point-by-point plot of the peaks function using z values
%      to determine the colors of the points
%
% Ken Garrard, North Carolina State University, 2005
% Based on plot3c by Uli Theune, University of Alberta

function g = plot3k(L,c,marker,nlabels)

error(nargchk(1,4,nargin));
if nargin < 2 || isempty(c),       c       = L(:,3); end    % color by z values
if nargin < 3 || isempty(marker),  marker  = '.';    end    % '.' marker
if nargin < 4 || isempty(nlabels), nlabels = 10;     end    % 10 colorbar labels

% check for errors in input arguments
if size(L,2) ~= 3
   error('Location vector must have 3 columns');
end

if length(L) ~= length(c)
   error('Location vector and color vector must be the same length');
end

marker_str = '+o*.xsd^v><ph';
if (length(marker) ~= 1) || isempty(strfind(marker_str,marker))
   error('Invalid marker character, select one of ''%s''', marker_str);
end

% find color limits and range
min_c   = min(c);
max_c   = max(c);
range_c = max_c - min_c;

% get current colormap
cmap = colormap;
clen = length(cmap);

% calculate color map index for each point
L(:,4) = min(max(round((c-min_c)*(clen-1)/range_c),1),clen);

% sort by color map index
L = sortrows(L,4);

% build index vector of color transitions (last point for each color)
dLix = [find(diff(L(:,4))>0); length(L)];

% plot data points in groups by color map index
hold on;                           % add points to one set of axes
s = 1;                             % index of 1st point in a color group
for k = 1:length(dLix)             % loop over each non-empty color group
   plot3(L(s:dLix(k),1), ...       % call plot3 once for each color group
         L(s:dLix(k),2), ...
         L(s:dLix(k),3), ...
         marker,         ...
         'MarkerEdgeColor',cmap(L(s,4),:), ... % same marker color from cmap
         'MarkerFaceColor',cmap(L(s,4),:) );   %   for all points in group
   s = dLix(k)+1;                  % next group starts at next point
end
hold off;

% set plot characteristics
view(3);                           % use default 3D view, (-37.5,30)
grid on;
set(gca,'FontName','Arial','FontSize',10,'FontWeight','bold');

% format the colorbar
h = colorbar;
nlabels = abs(nlabels);                    % number of labels must be positive
set(h,'YLim',[1 clen]);                    % set colorbar limits
set(h,'YTick',linspace(1,clen,nlabels));   % set tick mark locations

% create colorbar tick labels based on color vector data values
tick_vals = linspace(min_c,max_c,nlabels);

% create cell array of color bar tick label strings
warning off MATLAB:log:logOfZero;
for i = 1:nlabels
   if abs(min(log10(abs(tick_vals)))) <= 3, fm = '%-4.3f';   % fixed
   else                                     fm = '%-4.2E';   % floating
   end
   labels{i} = sprintf(fm,tick_vals(i));
end
warning on MATLAB:log:logOfZero;

% set tick label strings
set(h,'YTickLabel',labels,'FontName','Arial','FontSize',9,'FontWeight','bold');

g = gcf;     % return figure handle
