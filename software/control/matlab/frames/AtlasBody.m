classdef AtlasBody < LCMCoordinateFrame & Singleton
  % coordinate frame for the atlas model's rigid bodies---one coordinate per body
  % potentially useful for designating what bodies are supporting, etc.
  methods
    function obj=AtlasBody(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      obj = obj@LCMCoordinateFrame('SupportBodies',r.getNumBodies(),'x');
      obj = obj@Singleton();
      
      if isempty(obj.lcmcoder)
        link_names =  r.getLinkNames();
        coder = drc.control.RobotBodyCoder('atlas', link_names);
        setLCMCoder(obj,JLCMCoder(coder));
        obj.setCoordinateNames(link_names);
        obj.setDefaultChannel('ACTIVE_SUPPORTS');
      end
    end
  end
end
