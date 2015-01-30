classdef AtlasPositionControllerData < ControllerData

  % properties that can be modified 'on the fly'
  properties (SetAccess=public,GetAccess=public)
    qtraj % generalize configuration vector or trajectory 
  end
  
  methods 
    function obj = AtlasPositionControllerData(data)
      typecheck(data,'struct');
      obj = obj@ControllerData(data);
    end
 
    function data=verifyControllerData(~,data)
      assert(isa(data.qtraj,'Trajectory') || isnumeric(data.qtraj));      
    end
        
    function updateControllerData(obj,data)
      if isfield(data,'qtraj')
        obj.qtraj = data.qtraj;
      end
    end
  end
end