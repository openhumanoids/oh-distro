classdef Info
  
  properties (Constant)
    SUCCESS = 1
    FAIL_KINEMATIC_CONSTRAINT = 11
    FAIL_COLLISION = 12
    FAIL_TOO_MANY_ITERATIONS = 13
    FAIL_OTHER = 14
    FAIL_NO_FINAL_POSE = 15
  end
  
  properties
    status
    failingConfiguration
    infeasibleConstraints
    finalPoseTime
    reachingTime
    improvingTime
    shortcutTime
    rebuildTime
    reachingNpoints
    improvingNpoints
    shortcutNpoints
    rebuildNpoints
    IKTime
    IKFinalPoseTime
    IKReachingTime
    IKImprovingTime
    IKShortcutTime
    IKRebuildTime
    collisionTime
    collisionFinalPoseTime
    collisionReachingTime
    collisionImprovingTime
    collisionShortcutTime
    costReaching
    costImproving
    costShortcut
    nPoints
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
    
    function obj = setStatus(obj, status)
      obj.status = status;
    end
    
    function res = eq(obj, val)
      res = [obj(:).status] == val;
    end
    
    function res = ne(obj, val)
      res = [obj(:).status] ~= val;
    end
    
  end
  
end