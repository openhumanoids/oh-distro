function exerciseLadderPlanner(x0_msg,footstep_msg)
  lc = lcm.lcm.LCM.getSingleton();
  lc.publish('EST_ROBOT_STATE',drc.robot_state_t(x0_msg));
  pause(1);
  lc.publish('APPROVED_FOOTSTEP_PLAN',drc.deprecated_footstep_plan_t(footstep_msg));
end
