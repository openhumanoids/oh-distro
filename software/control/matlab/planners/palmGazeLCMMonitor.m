classdef palmGazeLCMMonitor
  % NOTEST
  %
  % Class for monitoring incoming LCM messages related to the Drill task
  %  current responsibilities:
  %    -Get current state estimate
  %    -Get drill affordance information
  %    -Get wall affordance information
  properties
    lc
    left_palm_monitor
    right_palm_monitor
    atlas
    state_frame
  end
  
  
  methods
    function obj = palmGazeLCMMonitor(atlas)
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.left_palm_monitor = drake.util.MessageMonitor(drc.ee_goal_t(), 'utime');
      obj.right_palm_monitor = drake.util.MessageMonitor(drc.ee_goal_t(), 'utime');
      obj.lc.subscribe('RIGHT_PALM_SIMPLE_GAZE_GOAL',obj.right_palm_monitor);
      obj.lc.subscribe('LEFT_PALM_SIMPLE_GAZE_GOAL',obj.left_palm_monitor);
      
      obj.atlas = atlas;
      obj.state_frame = atlas.getStateFrame;
      obj.state_frame.subscribe('EST_ROBOT_STATE');
    end
    
    function pose = getLeftPalmPose(obj)
      data = obj.left_palm_monitor.getNextMessage(0); %do not wait
      if isempty(data)
        pose = [];
        return;
      end
      goal = drc.ee_goal_t(data);
      pose = [goal.ee_goal_pos.translation.x;
        goal.ee_goal_pos.translation.y
        goal.ee_goal_pos.translation.z
        goal.ee_goal_pos.rotation.w
        goal.ee_goal_pos.rotation.x
        goal.ee_goal_pos.rotation.y
        goal.ee_goal_pos.rotation.z];    end
    
    function pose = getRightPalmPose(obj)
      data = obj.right_palm_monitor.getNextMessage(0); %do not wait
      if isempty(data)
        pose = [];
        return;
      end
      goal = drc.ee_goal_t(data);
      pose = [goal.ee_goal_pos.translation.x;
        goal.ee_goal_pos.translation.y
        goal.ee_goal_pos.translation.z
        goal.ee_goal_pos.rotation.w
        goal.ee_goal_pos.rotation.x
        goal.ee_goal_pos.rotation.y
        goal.ee_goal_pos.rotation.z];
    end
    
    function q = getStateEstimate(obj)
      [lb, ub] = obj.atlas.getJointLimits;
      [x,~] = getNextMessage(obj.state_frame,10);
      while (isempty(x))
        [x,~] = getNextMessage(obj.state_frame,10);
      end
      q = x(1:34);
      
      if max(q - ub) > -1e-3,
        q = min(q,ub);
      end
      if max(lb - q) > -1e-3,
        q = max(q,lb);
      end
    end
  end
  
end