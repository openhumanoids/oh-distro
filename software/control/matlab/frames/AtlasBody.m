classdef AtlasBody < LCMCoordinateFrameWCoder & Singleton
  % coordinate frame for the atlas model's rigid bodies---one coordinate per body
  % potentially useful for designating what bodies are supporting, etc.
  methods
    function obj=AtlasBody(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      link_names =  r.getLinkNames();
      coder = RobotBodyCoder('atlas', link_names);
      
      obj = obj@LCMCoordinateFrameWCoder('SupportBodies',r.getNumBodies(),'x',JLCMCoder(coder));
      obj.setCoordinateNames(link_names);
      obj.setDefaultChannel('ACTIVE_SUPPORTS');
    end
  end
end
