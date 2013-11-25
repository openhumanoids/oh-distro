classdef ManipPlannerConfigPublisher
  properties
    lc;
    channel;
  end
  
  methods
    function obj = ManipPlannerConfigPublisher(channel)
      obj.channel = channel;
      obj.lc = lcm.lcm.LCM.getSingleton();
    end
    
    function publish(obj, utime, data)
      msg = drc.planner_config_t();
      msg.utime = utime;
      msg.desired_ee_arc_speed = data.desired_ee_arc_speed;
      msg.desired_joint_speed = data.desired_joint_speed;
      msg.plan_execution_time = data.plan_execution_time;
      msg.active_planner = data.planner;
      if(data.planner == drc.planner_config_t.REACHING_PLANNER) 
        if(data.planning_mode == 1)
          msg.reaching_mode= drc.manip_plan_control_t.IKSEQUENCE_ON;
        elseif(data.planning_mode == 2)
          msg.reaching_mode= drc.manip_plan_control_t.IKSEQUENCE_OFF;
        elseif(data.planning_mode == 3)
          msg.reaching_mode= drc.manip_plan_control_t.TELEOP;
        elseif(data.planning_mode == 4)
          msg.reaching_mode= drc.manip_plan_control_t.FIXEDJOINTS;
        else
          error('mode not supported');
        end
      elseif(data.planner == drc.planner_config_t.MANIPULATION_PLANNER)
        if(data.planning_mode == 1)
          msg.manip_mode= drc.manip_plan_control_t.IKSEQUENCE_ON;
        elseif(data.planning_mode == 2)
          msg.manip_mode= drc.manip_plan_control_t.IKSEQUENCE_OFF;
        elseif(data.planning_mode == 3)
          msg.manip_mode= drc.manip_plan_control_t.TELEOP;
        elseif(data.planning_mode == 4)
          msg.manip_mode= drc.manip_plan_control_t.FIXEDJOINTS;
        else
          error('mode not supported');
        end
      end
      
      obj.lc.publish(obj.channel, msg);
    end
  end
end
