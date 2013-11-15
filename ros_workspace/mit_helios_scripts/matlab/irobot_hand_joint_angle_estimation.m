function irobot_hand_joint_angle_estimation()

data = import_irobot_hand_data('../data/joint_angle_estimation_data.csv');

xData = data.motorHallEncoder0;
yData = data.proximalJointAngle0;

validIndices = find(xData > 0);
xData = xData(validIndices);
yData = yData(validIndices);

nKnotPoints = 3;
[param0, xKnot1, xKnotFinal] = getParam0(xData, yData, nKnotPoints);

[lb, ub] = getbounds(param0, xKnot1, xKnotFinal, nKnotPoints);

  function y = piecewiseLinear(param, xDatai)
    xKnot = [xKnot1; param(1 : nKnotPoints - 2); xKnotFinal];
    yKnot = param(nKnotPoints - 1 : end);    
    y = interp1(xKnot, yKnot, xDatai, 'linear');
  end

param = lsqcurvefit(@piecewiseLinear, param0, xData, yData, lb, ub);

plot(xData, yData, 'b', xData, piecewiseLinear(param, xData), 'r');

xKnotSol = [xKnot1; param(1 : nKnotPoints - 2); xKnotFinal];
yKnotSol = param(nKnotPoints - 1 : end);

disp('xKnotSol:');
disp(num2str(xKnotSol));

disp('yKnotSol:');
disp(num2str(yKnotSol));

end

function [x0, xKnot1, xKnotFinal] = getParam0(xData, yData, nKnotPoints)
[xKnot1, minIndex] = min(xData);
[xKnotFinal, maxIndex] = max(xData);

yDataKnot1 = yData(minIndex);
yDataKnotFinal = yData(maxIndex);

xKnot = [xKnot1; xKnotFinal];
yDataKnot = [yDataKnot1; yDataKnotFinal];

xs = linspace(xKnot1, xKnotFinal, nKnotPoints)';
ys = interp1(xKnot, yDataKnot, xs, 'linear');

x0 = [xs(2 : end - 1); ys];
end

function [lb, ub] = getbounds(param0, xKnot1, xKnotFinal, nKnotPoints)
xs = [xKnot1; param0(1 : nKnotPoints - 2); xKnotFinal];
ys = param0(nKnotPoints - 1 : end);

xlb = xs(1 : end - 2);
xub = xs(3 : end);

lb = [xlb; -inf * ones(size(ys))];
ub = [xub; inf * ones(size(ys))];

end