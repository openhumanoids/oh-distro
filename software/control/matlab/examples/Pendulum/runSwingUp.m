function runSwingUp()
% runs trajectory optimization and animates open-loop playback

options.plan_publisher = drc.control.RobotPlanPublisher('Pendulum',{'theta'},'CANDIDATE_ROBOT_PLAN');

addpath([getDrakePath(),'/examples/Pendulum']);
pd = PendulumPlant;
[utraj,xtraj] = swingUpTrajectory(pd,options);
t=xtraj.getBreaks();

%pp.publish(t,xtraj.eval(t));

%runLCM(utraj);

end
