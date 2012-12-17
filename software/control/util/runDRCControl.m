function runDRCControl(robot,controller,robot_name,state_channel,cmd_channel,options)
%
% @param robot Dynamical system to be controlled
% @param sys Dynamical System to run as a controller (visualizers work,
% too)
% @param robot_name
% @param state_channel name of the LCM channel to listen for robot_state_t
% messages on.
% @param cmd_channel LCM channel name on which to publish actuator_cmd_t messages (if controller has outputs)

if(~isfield(options,'tspan'))
    options.tspan = [0,inf];
end

if (controller.getNumContStates()~=0)
    error('not implemented yet');  % but won't be too hard
end

nx = getNumStates(robot);
ndof = nx/2;
joint_names = robot.getStateFrame.coordinates(1:ndof);
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
%state_listener = RobotStateListener(robot_name,joint_names,state_channel);

lcmcoder = RobotStateCoder(robot_name, joint_names);
state_listener=LCMCoordinateFrameWCoder(robot_name,nx,robot.getStateFrame().prefix,lcmcoder);
state_listener.subscribe(state_channel);

if (getNumOutputs(controller)>0)
    cmd_names = robot.getInputFrame().coordinates;
    cmd_names = regexprep(cmd_names,'_motor','');     
    cmd_publisher = ActuatorCmdPublisher(robot_name,cmd_names,cmd_channel);
end

global g_scope_enable; g_scope_enable = true;

disp('Controller ready...');
% just run as fast as possible
t=options.tspan(1); tic;
while (t<=options.tspan(2))
  [x,ts] = getNextMessage(state_listener,1);
  if (~isempty(x))
    %t = ts/1000000
    %x'
    u = controller.output(t,[],x);
    %u = zeros(robot.getNumInputs(),1);%controller.output(t,[],x);
    %u'
    %u =zeros(getNumInputs(robot),1);
    %u(7) = 100;
    %u(20) = -100;

    
    %fprintf('time is %f\n',t);
    %fprintf('timestamp is %f\n',getLastTimestamp(state_listener));
    %fprintf('state is %f\n',x);
    %fprintf('control is %f\n\n',u);
    if (getNumOutputs(controller)>0)
      cmd_publisher.publish(u);
    end
    t = options.tspan(1)+toc;
  else
    t=options.tspan(1)+toc;
%    fprintf(1,'waiting... (t=%f)\n',t);
  end
end

