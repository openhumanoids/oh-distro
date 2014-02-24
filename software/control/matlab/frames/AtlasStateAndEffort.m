classdef AtlasStateAndEffort < LCMCoordinateFrame & Singleton
  
  methods
    function obj=AtlasStateAndEffort(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      obj = obj@LCMCoordinateFrame('AtlasStateAndEffort',r.getNumDOF*3,'x');
    
      if isempty(obj.lcmcoder)  % otherwise I had a singleton
        joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
        coder = drc.control.RobotStateCoder(joint_names,true);
      
        obj = setLCMCoder(obj,JLCMCoder(coder));
        coordinate_names = [joint_names;strcat(joint_names,'_dot');strcat(joint_names,'_effort')];
        obj.setCoordinateNames(coordinate_names);
        obj.setDefaultChannel('EST_ROBOT_STATE');
      end
    end
  end
end
