function DrawComponentsRow(rows, cols, subplotoffset, truevals, testunit, namestr)
%DRAWCOMPONENTSROW Summary of this function goes here
%   Detailed explanation goes here

 % Draw  components
    subplot(rows,3, subplotoffset*3+1);
    plot(truevals);
    grid on
    ylabel(namestr)
    subplot(rows,3, subplotoffset*3+2);
    plot(testunit);
    grid on
    subplot(rows,3, subplotoffset*3+3);
    plot(truevals - testunit);
    grid on


end

