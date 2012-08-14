function runDRCControl(sys,robot_name,state_channel,cmd_channel,options)
%
% @param sys Dynamical System to run as a controller (visualizers work,
% too)
% @param robot_name
% @param state_channel name of the LCM channel to listen for robot_state_t
% messages on.
% @param cmd_channel LCM channel name on which to publish actuator_cmd_t messages (if sys has outputs)

if(~isfield(options,'tspan'))
    options.tspan = [0,inf];
end

if (sys.getNumStates()~=0)
  error('not implemented yet');  % but won't be too hard
end

ndof = getNumInputs(sys)/2;
joint_name = sys.getInputFrame.coordinates(1:ndof);
state_listener = RobotStateListener(robot_name,joint_name,state_channel);

if (getNumOutputs(sys)>0)
    cmd_names=cellfun(@(a)a(1:end-6),sys.getOutputFrame().coordinates,'UniformOutput',false);  % temporary hack: remove the string "Torque" from the end of each string    
    cmd_publisher = ActuatorCmdPublisher(robot_name,cmd_names,cmd_channel);
end

global g_scope_enable; g_scope_enable = true;

% just run as fast as possible
t=options.tspan(1); tic;
while (t<=options.tspan(2))
  x = getNextMessage(state_listener,1000);
  if (~isempty(x))
    u = sys.output(t,[],x);
    if (getNumOutputs(sys)>0)
      cmd_publisher.publish(u);
    end
  else
    t=options.tspan(1)+toc;
    fprintf(1,'waiting... (t=%f)\n',t);
  end
end

