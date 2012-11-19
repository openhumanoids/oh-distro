function runDRCPlanning(sys,planFun,robot_name,state_channel,navGoal_channel,plan_channel,options)
% @param sys, a dynamical system, like Atlas
% @param planFun, a function handle, takes the current state and navigation
% goal as input, outputs the state and control profile
if(~isfield(options,'tspan'))
    options.tspan = [0,inf];
end

state_listener = RobotStateListener(robot_name,joint_name,state_channel);
navGoal_listener = RobotNavGoalListener(robot_name,navGoal_channel);
t = options.tspan(1); tic;

while(t<=options.tspan(2))
    navGoal = getNextMessage(navGoal_listener,100);
    if(~(isempty(navGoal)))
        xCurr = getNextMessage(state_listener,10);
        fprintf('time is %f\n',t);
        fprintf('state is %f\n',xCurr);
        fprintf('navigation goal is %f\n\n',navGoal);
    else
        fprintf(1,'waiting... (t=%f)\n',t);
    end
    t = options.tspan(1)+toc;
end
end