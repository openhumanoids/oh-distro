function result = optimize_rotation(poses1,poses2)

problem.stepsize = 100;
problem.poses1 = poses1;
problem.poses2 = poses2;

opts = optimset('Display','iter');
x = lsqnonlin(@error_func, [0;0;0;1], [], [], opts, problem);
result.R = quat2rot(x(1:4));

function e = error_func(x,problem)

R = quat2rot(x(1:4));
e = zeros(numel(problem.poses1)*9,1);
counter = 1;
for i = 1:numel(problem.poses1)
    e_sub = problem.poses2(i).R*R' - problem.poses1(i).R;
    e(counter:counter+8) = e_sub;
    counter = counter+9;
end
