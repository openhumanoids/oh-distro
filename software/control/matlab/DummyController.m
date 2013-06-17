classdef DummyController < DRCController

  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end

  methods 
    function obj = DummyController(name,r,options)
      typecheck(r, 'Atlas');
      sys = DummySys(r,options);
      obj = obj@DRCController(name,sys,AtlasState(r));
      obj.robot = r;
      obj = addLCMTransition(obj,'START_STANDING',drc.recovery_t(),'standing');
      obj = addLCMTransition(obj,'WALKING_PLAN',drc.walking_plan_t(),'crawling');  % for crawling
      obj = addLCMTransition(obj,'BRACE_FOR_FALL',drc.utime_t(),'bracing');
      obj = addLCMTransition(obj,'START_SITUP_CONTROLLER', drc.utime_t(),'sit_up');
  end

    function msg = status_message(obj,t_sim,t_ctrl);
      msg = drc.controller_status_t();
      msg.utime = t_sim * 1e6;
      msg.state = msg.DUMMY;
      msg.controller_utime = t_ctrl * 1e6;
      msg.V = 0;
      msg.Vdot = 0;
    end

    function obj = initialize(obj, data)
    end
  end
end


