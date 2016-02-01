function runRelativeEndEffectorTrajPlanner(channel,delim,options)
  if nargin < 3
    options = struct();
  end
  lc = lcm.lcm.LCM.getSingleton();
  status_code = 6;
  waiting = true;

  r = DRCAtlas();
  nq = r.getNumPositions();
  load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_bdi_fp.mat'));
  qstar = xstar(1:nq);
  q0 = qstar;
  joint_names = r.getStateFrame.coordinates(1:getNumPositions(r));
  robot_state_coder = LCMCoordinateFrame('AtlasState',JLCMCoder(drc.control.RobotStateCoder(joint_names)),'x');
  robot_state_coder.subscribe('EST_ROBOT_STATE');

  % Create lcm monitor and publisher
  msg_monitor = drake.util.MessageMonitor(drc.behavior_command_t(),'utime');
  lc.subscribe(channel,msg_monitor);
  plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
  % Main loop
  msg ='Relative EETraj Plan: Listening ...'; disp(msg); send_status(status_code,0,0,msg);
  while true
    % Waiting for message loop
    while waiting
      [x,~] = getNextMessage(robot_state_coder,10);
      if (~isempty(x))
        x0=x;
        q0 = x0(1:nq);
      end
      % Get message
      data = msg_monitor.getNextMessage(0);
      % Check if we got message. If so, we're done waiting
      if ~isempty(data)
        msg = drc._behavior_command_t(data);
        msg_str = char(msg.command);
        waiting = false;
      end
      pause(0.01);
    end
    msg ='Relative EETraj Plan: Received command ...'; disp(msg); send_status(status_code,0,0,msg);
    % Parse message string
    s = regexp(msg_str,delim,'split');
    traj_name = s{1};
    is_left_sided = strcmpi(s{2},'LEFT');
    logical2lr = @(b) regexprep(sprintf('%i',b),{'1','0'},{'l','r'});
    ee_name = [logical2lr(is_left_sided) '_hand'];

    % Load data from file
    [ee_pose_relative,T] = loadRelativeEEPoseTraj(traj_name,is_left_sided);

    % Generate plan
    msg ='Relative EETraj Plan: Computing plan ...'; disp(msg); send_status(status_code,0,0,msg);
    options.ref_link_name = ee_name;
    X_plan = relativeEndEffectorTrajPlanner(r,q0,ee_name,ee_pose_relative,options);

    % Publish plan
    msg ='Relative EETraj Plan: Publishing plan ...'; disp(msg); send_status(status_code,0,0,msg);
    plan_pub.publish([zeros(2,size(X_plan,2));X_plan],T,get_timestamp_now());
    waiting = true;
  end
end
