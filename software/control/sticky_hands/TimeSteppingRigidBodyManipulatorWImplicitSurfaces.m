classdef TimeSteppingRigidBodyManipulatorWImplicitSurfaces < TimeSteppingRigidBodyManipulator

  methods
    function obj = TimeSteppingRigidBodyManipulatorWImplicitSurfaces(varargin)
      obj@TimeSteppingRigidBodyManipulator(varargin{:});
    end
    
    function obj = setManipulandParams(obj,params)
      obj.manip = setManipulandParams(obj.manip,params);
    end
  end
  
end