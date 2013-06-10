classdef AtlasState < LCMCoordinateFrameWCoder & Singleton
  
  methods
    function obj=AtlasState(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      coder = RobotStateCoder('atlas', joint_names);
      
      obj = obj@LCMCoordinateFrameWCoder('AtlasState',r.getNumStates(),'x',JLCMCoder(coder));
      obj.setCoordinateNames(r.getStateFrame.coordinates);
      obj.setDefaultChannel('EST_ROBOT_STATE');
      
%      if (obj.mex_ptr==0)
%        obj.mex_ptr = RobotStateMonitor('atlas',joint_names);
%      end
    end

%     function delete(obj)
%       RobotStateMonitor(obj.mex_ptr);
%     end
%     
%     function obj = subscribe(obj,channel)
%       RobotStateMonitor(obj.mex_ptr,0,channel);
%     end
%     
%     function [x,t] = getNextMessage(obj,timeout)   % x=t=[] if timeout
%       [x,t] = RobotStateMonitor(obj.mex_ptr,1,timeout);
%     end
%     
%     function [x,t] = getMessage(obj)
%       [x,t] = RobotStateMonitor(obj.mex_ptr,2);
%     end
%     
%     function [x,t] = getCurrentValue(obj)
%       [x,t] = RobotStateMonitor(obj.mex_ptr,2);
%     end
%     
%     function t = getLastTimestamp(obj)
%       t = RobotStateMonitor(obj.mex_ptr,3);
%     end
%     
%     function markAsRead(obj)
%       RobotStateMonitor(obj.mex_ptr,4);
%     end
  end
  
  properties
    mex_ptr=0
  end
end
