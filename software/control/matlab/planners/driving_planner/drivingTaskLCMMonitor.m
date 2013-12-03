classdef drivingTaskLCMMonitor
  % NOTEST
  %
  % Class for monitoring incoming LCM messages related to the Drill task
  %  current responsibilities:
  %    -Get current state estimate
  %    -Get drill affordance information
  %    -Get wall affordance information
  properties
    lc
    affordance_monitor
    driving_control_monitor
    atlas
    state_frame
  end
  
  methods
    function obj = drivingTaskLCMMonitor(atlas)
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.affordance_monitor = drake.util.MessageMonitor(drc.affordance_collection_t(), 'utime');
      obj.lc.subscribe('AFFORDANCE_COLLECTION',obj.affordance_monitor);
      
      obj.driving_control_monitor = drake.util.MessageMonitor(drc.driving_control_cmd_t(),'utime');
      obj.lc.subscribe('DRIVING_CONTROL',obj.driving_control_monitor);
      
      obj.atlas = atlas;
      obj.state_frame = atlas.getStateFrame;
      obj.state_frame.subscribe('EST_ROBOT_STATE');
    end
    
    function [data] = getDrivingControlMsg(obj, timeout)
      if nargin < 2
        timeout = 2000; % default
      end
      lcm_data = obj.driving_control_monitor.getNextMessage(timeout);
      data = [];
      if isempty(lcm_data)
        type = -1;
        return;
      end
      
      msg = drc.driving_control_cmd_t(lcm_data);
      data.steering = msg.steering_angle;
      data.ankle = msg.throttle_value;
    end
    
    function valve_data = getValveAffordance(obj)
      data = obj.affordance_monitor.getNextMessage(2000); % default timeout
      if isempty(data)
        valve_data = [];
        return
      end
      
      aff_collection = drc.affordance_collection_t(data);
      
      valve_aff = obj.getAffordanceByOTDFType(aff_collection, 'steering_cyl');
      if isempty(valve_aff)
        valve_data = [];
      else
        valve_data = obj.parseValveData(valve_aff);
      end
    end
    
    function q = getStateEstimate(obj)
      [lb, ub] = obj.atlas.getJointLimits;
      [x,~] = getMessage(obj.state_frame);
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
  
  
  methods (Access = private)
    
    
    function valve_data = parseValveData(obj, valve_aff)
      R = rpy2rotmat(valve_aff.origin_rpy);
      valve_data.normal = -R(:,3);
      valve_data.radius = valve_aff.params(3);% - .07; 
      valve_z = [0;0;1] - [0;0;1]'*valve_data.normal*valve_data.normal;
      valve_z = valve_z/norm(valve_z);
      valve_data.center = valve_aff.origin_xyz;%  + .10*valve_data.normal;
      valve_data.init_pt = valve_data.center + valve_z*valve_data.radius;
    end
    
    function aff = getAffordanceByOTDFType(obj, collection, type)
      aff = [];
      for i=1:collection.naffs,
        if strcmp(collection.affs(i).otdf_type,type)
          aff = collection.affs(i);
          return
        end
      end
    end
  end
  
end