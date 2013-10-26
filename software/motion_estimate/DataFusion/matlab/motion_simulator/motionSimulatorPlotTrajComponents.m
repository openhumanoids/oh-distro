function motionSimulatorPlotTrajComponents( varargin )
%MOTIONSIMULATORPLOTTRAJ3D Summary of this function goes here
%   Detailed explanation goes here


% linesettings = {'-','--','.-'};

figure(varargin{1}), clf;

truepose = varargin{2};

rows = 5;
cols = 3;

for n =3:length(varargin)
    figure(varargin{1}+(n-3)), clf;

    drawpose = varargin{n};

    
    % Draw position components
    DrawComponentsRow(rows, cols, 1, truepose.P_l, drawpose.P_l, 'Position');
    
    % Draw velocity components
    DrawComponentsRow(rows, cols, 2, truepose.V_l, drawpose.V_l, 'Velocity');
    
    % Draw acceleration components
    DrawComponentsRow(rows, cols, 3, truepose.f_l, drawpose.f_l, 'Local Resolved Acc');
    
    % Draw acceleration components
    DrawComponentsRow(rows, cols, 4, truepose.a_l, drawpose.a_l, 'Local Acceleration');
    
    % Draw acceleration components
    DrawComponentsRow(rows, cols, 5, truepose.E, drawpose.E, 'Euler Angles');
    
end


% title(['3D Position from ' num2str(t(1)) ' s to ' num2str(t(end)) ' s'])

