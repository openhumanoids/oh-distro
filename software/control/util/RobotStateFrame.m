classdef RobotStateFrame < CoordinateFrame & LCMSubscriber & LCMPublisher
  
  methods
    function obj=RobotStateFrame(robot_name,joint_name)
      typecheck(robot_name,'char');
      typecheck(joint_name,'cell');
      
      coordinates = vertcat(joint_name,cellfun(@(a) [a,'dot'],joint_name,'UniformOutput',false));
      obj = obj@CoordinateFrame([robot_name,'State'],numel(joint_name)*2,'x',coordinates);

      obj.subscriber = RobotStateListener(robot_name,joint_name);
      obj.publisher = RobotStatePublisher(robot_name,joint_name);
    end
  
    function obj = subscribe(obj,channel)
      obj.subscriber.subscribe(channel);
    end
    
    function [x,t] = getNextMessage(obj,timeout)   % x=t=[] if timeout
      x = obj.subscriber.getNextMessage(timeout);
      if isempty(x)
        t=[];
      else
        t = obj.subscriber.getLastTimestamp();
      end
    end
    
    function [x,t] = getCurrentValue(obj)
      x = obj.subscriber.getState();
      t = obj.subscriber.getLastTimestamp();
    end
    
    function publish(obj,t,x,channel)
      obj.publisher.publish(t,x,channel);
    end
    
    function str = defaultChannel(obj)
      str = 'EST_ROBOT_STATE';
%      str = obj.name;
    end
  
  end
  
  properties
    subscriber;
    publisher;
  end
end