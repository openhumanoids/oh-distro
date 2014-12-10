function motionSimulatorPlotTrajComponents( varargin )
%MOTIONSIMULATORPLOTTRAJ3D Summary of this function goes here
%   Detailed explanation goes here


% linesettings = {'-','--','.-'};

figure(varargin{1}), clf;

truepose = varargin{2};

rows = 7;
cols = 3;

for n =3:length(varargin)
    figure(varargin{1}+(n-3)), clf;

    drawpose = varargin{n};

    % Draw body rotation rate components
    DrawComponentsRow(rows, cols, 1, truepose.w_b, truepose.w_b, 'Body rotations');
    
    % Draw Euler angle rotatoins
    DrawComponentsRow(rows, cols, 2, truepose.E, drawpose.E, 'Euler Angles');
    
     % Draw acceleration components
    DrawComponentsRow(rows, cols, 3, truepose.a_b, truepose.a_b, 'Body Accelerations');
    
    % Draw position components
    DrawComponentsRow(rows, cols, 7, truepose.P_l, drawpose.P_l, 'Position');
    
    % Draw velocity components
    DrawComponentsRow(rows, cols, 6, truepose.V_l, drawpose.V_l, 'Velocity');
    
    % Draw acceleration components
    DrawComponentsRow(rows, cols, 5, truepose.f_l, drawpose.f_l, 'Local Resolved Acc');
    
    % Draw acceleration components
    DrawComponentsRow(rows, cols, 4, truepose.a_l, drawpose.a_l, 'Local Accelerations');
    
    
    
end


% title(['3D Position from ' num2str(t(1)) ' s to ' num2str(t(end)) ' s'])

