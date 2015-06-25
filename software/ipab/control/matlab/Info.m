classdef Info
  
  properties (Constant)
    SUCCESS = 1
    FAIL_KINEMATIC_CONSTRAINT = 2
    FAIL_COLLISION = 3
    FAIL_TOO_MANY_ITERATIONS = 4
    FAIL_OTHER = 5
    NEW_POINT_ADDED = 6
  end
  
  properties
    status
    failingConfiguration
    infeasibleConstraints
  end
  
  methods
    function obj = Info(status, failingConfiguration, infeasibleConstraints)
      if nargin < 3, infeasibleConstraints = {}; end
      if nargin < 2, failingConfiguration = []; end
      
      obj.status = status;
      obj.failingConfiguration = failingConfiguration;
      obj.infeasibleConstraints = infeasibleConstraints;
    end
    
    function constProps = getConstProps(obj)
      mc = metaclass(obj);
      names = {mc.PropertyList.Name};
      constProps = names([mc.PropertyList.Constant]);
    end
    
    function status = getStatus(obj)
      constProps = obj.getConstProps();
      status = constProps{obj.status};
    end
    
    function res = eq(obj, val)
      res = [obj(:).status] == val;
    end
    
    function res = ne(obj, val)
      res = [obj(:).status] ~= val;
    end
    
  end
  
end