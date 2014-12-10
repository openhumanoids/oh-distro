function motionSimulatorPlotTraj3D( varargin )
%MOTIONSIMULATORPLOTTRAJ3D Summary of this function goes here
%   Detailed explanation goes here


colors = {'l','g:','r','k','m'};


figure(varargin{1}), clf;


for n =2:length(varargin)
    drawpose = varargin{n};
    plot3(drawpose.P_l(:,1),drawpose.P_l(:,2),drawpose.P_l(:,3),colors{mod(n,length(colors))},'Linewidth',2);
    hold on
end

grid on
% title(['3D Position from ' num2str(t(1)) ' s to ' num2str(t(end)) ' s'])
axis equal
