classdef AtlasCoordinates < LCMCoordinateFrame & Singleton
  % atlas q
  methods
    function obj=AtlasCoordinates(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      nq = r.getNumDOF();
      obj = obj@LCMCoordinateFrame('AtlasCoordinates',nq,'x'); 
      obj = obj@Singleton();

      if isempty(obj.lcmcoder)
        joint_names = r.getStateFrame.coordinates(1:nq); 
        coder = drc.control.JointAnglesCoder('atlas',joint_names);
        setLCMCoder(obj,JLCMCoder(coder));

        obj.setCoordinateNames(joint_names);
        obj.setDefaultChannel('DESIRED_ACCELERATION');
      end
    end
  end
end
