function motionSimulatorPlotTrajComponents( varargin )
%MOTIONSIMULATORPLOTTRAJ3D Summary of this function goes here
%   Detailed explanation goes here


% linesettings = {'-','--','.-'};

figure(varargin{1}), clf;

truepose = varargin{2};

rows = 4;
cols = 3;

for n =3:length(varargin)
    figure(varargin{1}+(n-3)), clf;
    
    % Draw position components
    drawpose = varargin{n};
    subplot(rows,3, 0*3+1);
    plot(truepose.P_l);
    grid on
    title('true')
    ylabel('Postion')
    subplot(rows,3, 0*3+2);
    plot(drawpose.P_l);
    grid on
    title('test unit')
    subplot(rows,3, 0*3+3);
    plot(truepose.P_l - drawpose.P_l);
    grid on
    title('residual')
    
%     % Draw velocity components
    subplotoffset = 1;
%     drawpose = varargin{n};
%     subplot(rows,3, subplotoffset*3+1);
%     plot(truepose.V_l);
%     grid on
%     ylabel('Velocity')
%     subplot(rows,3, subplotoffset*3+2);
%     plot(drawpose.V_l);
%     grid on
%     subplot(rows,3, subplotoffset*3+3);
%     plot(truepose.V_l - drawpose.V_l);
%     grid on
    DrawComponentsRow(rows, cols, subplotoffset, truepose.V_l, drawpose.V_l, 'Velocity');
    
    % Draw acceleration components
    subplotoffset = 2;
%     drawpose = varargin{n};
%     subplot(rows,3, subplotoffset*3+1);
%     plot(truepose.a_l);
%     grid on
%     ylabel('Local Acceleration')
%     subplot(rows,3, subplotoffset*3+2);
%     plot(drawpose.a_l);
%     grid on
%     subplot(rows,3, subplotoffset*3+3);
%     plot(truepose.a_l - drawpose.a_l);
%     grid on
    
    DrawComponentsRow(rows, cols, subplotoffset, truepose.a_l, drawpose.a_l, 'Local Acceleration');
    
end


% title(['3D Position from ' num2str(t(1)) ' s to ' num2str(t(end)) ' s'])

