classdef ManipPlannerConfigPublisher
  properties
    lc;
    channel;
  end
  
  methods
    function obj = ManipPlanPublisher(channel)
      obj.channel = channel;
      obj.lc = lcm.lcm.LCM.getSingleton();
    end
    
    function publish(obj, utime, data)
      msg = drc.planner_config_t();
      msg.utime = utime;
      msg.desired_ee_arc_speed = data.desired_ee_arc_speed;
      msg.desired_joint_speed = data.desired_joint_speed;
      msg.plan_execution_time = data.plan_execution_time;
      if(data.planning_mode == 1)
        msg.planning_mode = drc.manip_plan_control_t.IKSEQUENCE_ON;
      elseif(data.planning_mode == 2)
        msg.planning_mode = drc.manip_plan_control_t.IKSEQUENCE_OFF;
      elseif(data.planning_mode == 3)
        msg.planning_mode = drc.manip_plan_control_t.TELEOP;
      elseif(data.planning_mode == 4)
        msg.planning_mode = drc.manip_plan_control_t.FIXEDJOINTS;
      else
        error('Invalid planning mode');
      end
      
      obj.lc.publish(obj.channel, msg);
    end
  end
end