function runDRCPlanning(sys,planFun,robot_name,state_channel,navGoal_channel,plan_channel,options)
% @param sys, a dynamical system, like Atlas
% @param planFun, a function handle, takes the current state and navigation
% goal as input, outputs the state and control profile
if(~isfield(options,'tspan'))
    options.tspan = [0,inf];
end
planFunOptions = struct();
if(isfield(options,'sys2D'))
    planFunOptions = struct('sys2D',options.sys2D);
end
ndof = getNumStates(sys)/2;
joint_name = sys.getStateFrame.coordinates(1:ndof);
joint_name = regexprep(joint_name,'_','\.','preservecase');
[~,base_name] = regexp(joint_name(1),'(\w*)(\.x)','match','tokens');
base_name = base_name{1}{1}{1};
for i = 1:6
    joint_name(i) = regexprep(joint_name(i),base_name,'base','preservecase');
end
state_listener = RobotStateListener(robot_name,joint_name,state_channel);
navGoal_listener = RobotNavGoalListener('wheeled_atlas',navGoal_channel);
t = options.tspan(1); tic;

while(t<=options.tspan(2))
    navGoal = getNextMessage(navGoal_listener,10);
    if(~(isempty(navGoal)))
        xCurr = getNextMessage(state_listener,100);
        fprintf('time is %f\n',t);
        fprintf('state is %f\n',xCurr);
        fprintf('navigation goal is %f\n\n',navGoal);
        [tbreaks,xbreaks,utraj2D,xtraj2D,ltraj2D] = planFun(sys,navGoal,xCurr,planFunOptions);
        plan_publisher = RobotPlanPublisher(robot_name,length(tbreaks),joint_name(7:end),plan_channel);
        plan_publisher.publish(tbreaks,xbreaks);
        break;
    else
        fprintf(1,'waiting... (t=%f)\n',t);
    end
    t = options.tspan(1)+toc;
end


end