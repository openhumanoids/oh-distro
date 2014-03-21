function echoPlanAsRobotState(x0)
  lc = lcm.lcm.LCM.getSingleton();
  plan_monitor = drake.util.MessageMonitor(drc.robot_plan_t(),'utime');
  state_monitor = drake.util.MessageMonitor(drc.robot_state_t(),'utime');
  lc.subscribe('COMMITTED_ROBOT_PLAN',plan_monitor);
  lc.subscribe('EST_ROBOT_STATE',state_monitor);
  Fs = 30;
  if nargin < 1
    msg = state_monitor.getNextMessage(1);
    if isempty(msg)
      S = load([getenv('DRC_BASE'), '/software/control/matlab/data/atlas_bdi_fp.mat']);
      x0 = S.xstar;
    end
  end
  if nargin == 1 || isempty(msg)
    r = Atlas();
    joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
    robot_state_coder = LCMCoordinateFrame('AtlasState',JLCMCoder(drc.control.RobotStateCoder(joint_names)),'x');
    publish(robot_state_coder,0,x0,'EST_ROBOT_STATE');
    msg = state_monitor.getNextMessage(1);
  end
  current_robot_state = drc.robot_state_t(msg);

  display('Listening for committed plans ...');
  while true
    msg = plan_monitor.getNextMessage(0);

    if ~isempty(msg)
      display('Received plan. Echoing ...');
      robot_plan = drc.robot_plan_t(msg);
      lc.publish('EST_ROBOT_STATE',robot_plan.plan(1))
      for i = 2:robot_plan.num_states
        pause(1e-6*(robot_plan.plan(i).utime-robot_plan.plan(i-1).utime));
        current_robot_state.pose.translation.x = robot_plan.plan(i).pose.translation.x;
        current_robot_state.pose.translation.y = robot_plan.plan(i).pose.translation.y;
        current_robot_state.pose.translation.z = robot_plan.plan(i).pose.translation.z;
        current_robot_state.pose.rotation.w = robot_plan.plan(i).pose.rotation.w;
        current_robot_state.pose.rotation.x = robot_plan.plan(i).pose.rotation.x;
        current_robot_state.pose.rotation.y = robot_plan.plan(i).pose.rotation.y;
        current_robot_state.pose.rotation.z = robot_plan.plan(i).pose.rotation.z;
        current_robot_state.joint_position = robot_plan.plan(i).joint_position;

        current_robot_state.utime = get_timestamp_now();
        lc.publish('EST_ROBOT_STATE',current_robot_state)
      end
      current_robot_state = robot_plan.plan(end);
      display('Listening for committed plans ...');
    else
      pause(1/Fs);
      current_robot_state.utime = get_timestamp_now();
      lc.publish('EST_ROBOT_STATE',current_robot_state)
    end
  end
end
