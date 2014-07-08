classdef AtlasManipControllerData < ControllerData
  % Class that contains data needed by the Atlas position controller

    properties (SetAccess=public,GetAccess=public)
      qtraj % generalize configuration vector or trajectory 
      qddtraj % generalize acceleration trajectory 
      integral % position control integral term
      integral_gains
      integral_clamps
      enable_bdi_manip = false % determines whether BDI manip commands are computed 
      % from the plan and streamed to the robot
      firstplan = true
    end
  
  methods
    function obj = AtlasManipControllerData(data)
      typecheck(data,'struct');
      obj = obj@ControllerData(data);
    end
 
    function data=verifyControllerData(~,data)
      assert(isa(data.qtraj,'Trajectory') || isnumeric(data.qtraj));      
      assert(isnumeric(data.integral));
      assert(isnumeric(data.integral_gains));
      assert(isnumeric(data.integral_clamps));
      if isfield(data,'qddtraj')
        assert(isa(data.qddtraj,'Trajectory'));      
      end
      if isfield(data,'enable_bdi_manip')
        assert(islogical(data.enable_bdi_manip));      
      end
      if isfield(data,'firstplan')
        assert(islogical(data.firstplan));      
      end
    end
    
    function updateControllerData(obj,data)
      if isfield(data,'qtraj')
        obj.qtraj = data.qtraj;
      end
      if isfield(data,'qddtraj')
        obj.qddtraj = data.qddtraj;
      end
      if isfield(data,'integral')
        obj.integral = data.integral;
      end
      if isfield(data,'integral_gains')
        obj.integral_gains = data.integral_gains;
      end
      if isfield(data,'integral_clamps')
        obj.integral_clamps = data.integral_clamps;
      end
      if isfield(data,'enable_bdi_manip')
        obj.enable_bdi_manip = data.enable_bdi_manip;
      end
      if isfield(data,'firstplan')
        obj.firstplan = data.firstplan;
      end     
    end
  end
end