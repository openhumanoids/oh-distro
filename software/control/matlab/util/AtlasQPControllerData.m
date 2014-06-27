classdef AtlasQPControllerData < QPControllerData
  % contains QP control properties, plus integral gains etc. needed for
  % running on Atlas

  % optional: for properties that change infrequently or never
  properties (SetAccess=private,GetAccess=public)
    force_controlled_joints 
    position_controlled_joints
  end
  
  % properties that can be modified 'on the fly'
  properties (SetAccess=public,GetAccess=public)
    integral % position control integral terms
    integral_gains
    integral_clamps
    qd_int_state % state of the velocity integrator block
    enable_bdi_manip = false % determines whether BDI manip commands are computed 
    % from the plan and streamed to the robot
    firstplan = true
  end
  
  methods 
    function obj = AtlasQPControllerData(is_time_varying,data)
      typecheck(is_time_varying,'logical');
      typecheck(data,'struct');
   
      obj@QPControllerData(is_time_varying,data);
      
      data=verifyAtlasControllerData(obj,data);
      updateAtlasControllerData(obj,data);
    end
 
    function data=verifyAtlasControllerData(~,data)
      assert(isnumeric(data.force_controlled_joints));
      assert(isnumeric(data.position_controlled_joints));
      assert(isnumeric(data.integral));
      assert(isnumeric(data.integral_gains));
      assert(isnumeric(data.integral_clamps));
      if isfield(data,'qd_int_state')
        assert(isnumeric(data.qd_int_state));      
      end
      if isfield(data,'enable_bdi_manip')
        assert(islogical(data.enable_bdi_manip));      
      end
      if isfield(data,'firstplan')
        assert(islogical(data.firstplan));      
      end      
    end
    
    function updateAtlasControllerData(obj,data)
      if isfield(data,'force_controlled_joints')
        obj.force_controlled_joints = data.force_controlled_joints;
      end
      if isfield(data,'position_controlled_joints')
        obj.position_controlled_joints = data.position_controlled_joints;
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
      if isfield(data,'qd_int_state')
        obj.qd_int_state = data.qd_int_state;
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