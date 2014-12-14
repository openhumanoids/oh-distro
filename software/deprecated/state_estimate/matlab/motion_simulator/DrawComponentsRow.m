function DrawComponentsRow(rows, cols, subplotoffset, truevals, testunit, namestr)
%DRAWCOMPONENTSROW Summary of this function goes here
%   Detailed explanation goes here

 % Draw  components
    subplot(rows,3, (subplotoffset-1)*3+1);
    plot(truevals);
    title('true')
    grid on
    ylabel(namestr)
    subplot(rows,3, (subplotoffset-1)*3+2);
    plot(testunit);
    title('test unit')
    grid on
    subplot(rows,3, (subplotoffset-1)*3+3);
    plot(truevals - testunit);
    title('residual')
    grid on


end

