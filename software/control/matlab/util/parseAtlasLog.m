function [t_x,x_data,t_u,u_data,state_frame,input_frame] = parseAtlasLog(plant,logfile)
% function [t_x,x_data,t_u,u_data] = parseAtlasLog(plant,logfile)
%  Read a LCM log file and parse the robot state and commands for
%  an Atlas robot. The channels parsed are EST_ROBOT_STATE and
%  ATLAS_COMMAND.
%
%  NOTEST
%
%  @param plant The Atlas robot object
%  @param logfile The full file path of the log to open
%  @return t_x Time vector related to state data
%  @return x_data State data, each column relates to one measurement
%  @return t_u Time vector related to input data
%  @return u_data Control input data
use_java = 1;

% Java version is more than 10x faster
if use_java
%   parser = drc.control.AtlasStateCommandLogParser('EST_ROBOT_STATE', plant.getStateFrame.lcmcoder.jcoder,'ATLAS_COMMAND',plant.getInputFrame.lcmcoder.jcoder);
%   parser.parseLog(logfile);
%   t_x = parser.getTx();
%   t_u = parser.getTu();
%   x_data = reshape(parser.getStateData, [], length(t_x));
%   u_data = reshape(parser.getInputData, [], length(t_u));
    input_frame = AtlasPosVelTorqueRef(plant);
    state_frame = AtlasStateAndEffort(plant);
    
    parser = drc.control.LCMLogParser;
    parser.addChannel('EST_ROBOT_STATE',state_frame.lcmcoder.jcoder);
    parser.addChannel('ATLAS_COMMAND',input_frame.lcmcoder.jcoder);
    parser.parseLog(logfile);  
    t_x = parser.getT('EST_ROBOT_STATE');
    t_u = parser.getT('ATLAS_COMMAND');
    x_data = reshape(parser.getData('EST_ROBOT_STATE'), [], length(t_x));
    u_data = reshape(parser.getData('ATLAS_COMMAND'), [], length(t_u));
else
  warning('Deprecated, and not being updated');
  lcm_log = lcm.logging.Log(logfile,'r');
  state_hash = java.lang.String('EST_ROBOT_STATE').hashCode();
  control_hash = java.lang.String('ATLAS_COMMAND').hashCode();
  
  input_coder = plant.getInputFrame.lcmcoder;
  state_coder = plant.getStateFrame.lcmcoder;
  
  x_data = zeros(plant.getNumStates,5000);
  u_data = zeros(plant.getNumInputs,5000);
  t_x = zeros(5000,1);
  t_u = zeros(5000,1);
  
  state_ind = 0;
  input_ind = 0;
  while true
    try
      msg = lcm_log.readNext();
    catch e
      %     error(e)
      lcm_log.close();
      break
    end
    msg_hash = msg.channel.hashCode();
    if msg_hash == state_hash
      %     x_i = r.getStateFrame.lcmcoder.decode(drc.robot_state_t(msg.data));
      [x_i,t_i] = state_coder.decode(msg.data);
      if state_ind == size(x_data,2),
        x_data = [x_data zeros(size(x_data))];
        t_x = [t_x; zeros(size(t_x))];
      end
      state_ind = state_ind + 1;
      x_data(:,state_ind) = x_i;
      t_x(state_ind) = t_i;
    elseif msg_hash == control_hash
      [u_i,t_i] = input_coder.decode(msg.data);
      if input_ind == size(u_data,2),
        u_data = [u_data zeros(size(u_data))];
        t_u = [t_u; zeros(size(t_u))];
      end
      input_ind = input_ind + 1;
      u_data(:,input_ind) = u_i;
      t_u(input_ind) = t_i;
    end
  end
  
  x_data = x_data(:,1:state_ind);
  u_data = u_data(:,1:input_ind);
  t_x = t_x(1:state_ind);
  t_u = t_u(1:input_ind);
end
end