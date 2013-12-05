function irobot_hand_joint_angle_estimation()

data = import_irobot_hand_data('../data/joint_angle_estimation_data.csv');

motorHallEncoders = [data.motorHallEncoder0, data.motorHallEncoder1, data.motorHallEncoder2];
proximalJointAngles = [data.proximalJointAngle0, data.proximalJointAngle1, data.proximalJointAngle2];
distalJointAngles = [data.distalJointAngle0, data.distalJointAngle1, data.distalJointAngle2];

piecewiseLinearFit(motorHallEncoders, proximalJointAngles, 'motor tendon excursion', 'proximal joint angle', 3);
piecewiseLinearFit(motorHallEncoders, distalJointAngles, 'motor tendon excursion', 'distal joint angle', 3);

end

function piecewiseLinearFit(xData, yData, xName, yName, nKnotPoints)
validIndices = find(xData > 0);
xFitData = xData(validIndices);
yFitData = yData(validIndices);

[param0, xKnot1, xKnotFinal] = getParam0(xFitData, yFitData, nKnotPoints);

[lb, ub] = getbounds(param0, xKnot1, xKnotFinal, nKnotPoints);

param = lsqcurvefit(@piecewiseLinear, param0, xFitData, yFitData, lb, ub);

xKnotSol = [xKnot1; param(1 : nKnotPoints - 2); xKnotFinal];
yKnotSol = param(nKnotPoints - 1 : end);
slopes = diff(yKnotSol) ./ diff(xKnotSol);
yAxisIntercepts = yKnotSol(2 : end) - slopes .* xKnotSol(2 : end);

fprintf('xKnotSol:\n');
disp(xKnotSol);

fprintf('yKnotSol:\n');
disp(yKnotSol);

fprintf('slopes:\n')
disp(slopes);

fprintf('y axis intercepts:\n')
disp(yAxisIntercepts);


figure();
hold on;
for i = 1 : size(xData, 2)
    plot(xData(:, i), yData(:, i), 'b');
    plot(xData(:, i), piecewiseLinear(param, xData(:, i)), 'r');
end

xlabel(xName)
ylabel(yName)
hold off;

  function y = piecewiseLinear(param, xDatai)
    xKnot = [xKnot1; param(1 : nKnotPoints - 2); xKnotFinal];
    yKnot = param(nKnotPoints - 1 : end);    
    y = interp1(xKnot, yKnot, xDatai, 'linear');
  end
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