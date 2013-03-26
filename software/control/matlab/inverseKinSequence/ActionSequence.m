classdef ActionSequence
  % structure which lists a series of kinematic (and possibly dynamic)
  % objectives and constraints for a robot
  
  properties (Access=protected)
    %% Objectives
    
    %% Constraints
    kincon = {};  % a cell array of ActionKinematicConstraints
  end
  properties
    tspan=[inf, -inf];
    key_time_samples = []
  end
  
  methods
    
    function obj=addKinematicConstraint(obj,kc)
      typecheck(kc,'ActionKinematicConstraint');
      obj.kincon = [obj.kincon,{kc}];
      obj.tspan(1) = min(obj.tspan(1),kc.tspan(1));
      obj.tspan(2) = max(obj.tspan(2),kc.tspan(2));
      obj.key_time_samples = [obj.key_time_samples, kc.tspan(1) kc.tspan(2)];
      obj.key_time_samples = unique(obj.key_time_samples);
    end
    
    function ikargs = getIKArguments(obj,t)
      % I should detect the redundant constraint here
      ikargs={};
      for i=1:length(obj.kincon)
        ikargs=horzcat(ikargs,getIKArguments(obj.kincon{i},t));
      end
    end
  end
  
end

% NORELEASE
  