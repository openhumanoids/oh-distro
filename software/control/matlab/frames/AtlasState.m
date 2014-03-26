classdef AtlasState < LCMCoordinateFrame & Singleton
  
  methods
    function obj=AtlasState(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      obj = obj@LCMCoordinateFrame('AtlasState',r.getNumStates(),'x');
      obj = obj@Singleton();
      joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      if isempty(obj.lcmcoder)  % otherwise I had a singleton
        
        coder = drc.control.RobotStateCoder(joint_names);
      
        obj.setLCMCoder(JLCMCoder(coder));
        obj.setCoordinateNames(r.getStateFrame.coordinates);
        obj.setDefaultChannel('EST_ROBOT_STATE');
      end

    if (obj.mex_ptr==0)
      obj.mex_ptr = SharedDataHandle(RobotStateMonitor('atlas',joint_names));
    end
    end

     function delete(obj)
%       note that delete is also called on temporary objects used to
%       recover the actual Singleton object
%       if (obj.mex_ptr ~= 0)
%         RobotStateMonitor(obj.mex_ptr.data);
%       end
     end
     
      function obj = subscribe(obj,channel)
        chash = java.lang.String(channel).hashCode();
        if ~any(chash==obj.subscriptions)  % don't subscribe multiple times to the same channel
          RobotStateMonitor(obj.mex_ptr.data,0,channel);
          obj.subscriptions(end+1)=chash;
        end
      end
      
      function [x,t] = getNextMessage(obj,timeout)   % x=t=[] if timeout
        [x,t] = RobotStateMonitor(obj.mex_ptr.data,1,timeout);
      end
      
      function [x,t] = getMessage(obj)
        [x,t] = RobotStateMonitor(obj.mex_ptr.data,2);
      end
      
      function [x,t] = getCurrentValue(obj)
        [x,t] = RobotStateMonitor(obj.mex_ptr.data,2);
      end
      
      function t = getLastTimestamp(obj)
        t = RobotStateMonitor(obj.mex_ptr.data,3);
      end
      
      function markAsRead(obj)
        RobotStateMonitor(obj.mex_ptr.data,4);
      end
  end
  
 properties
  mex_ptr=0
 end
end
