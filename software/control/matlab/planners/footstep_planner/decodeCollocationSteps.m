function [steps, steps_rel] = decodeCollocationSteps(x)
x = reshape(x, 12, []);
steps = x(1:6,:);
steps_rel = x(7:12,:);
end
