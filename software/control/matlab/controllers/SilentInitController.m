classdef SilentInitController < DRCController

  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end

  methods 
    function obj = SilentInitController(name,r,options)
      typecheck(r, 'Atlas');
      if nargin < 3
        options=struct();
      end
      sys = DummySys(r,options);
      obj = obj@DRCController(name,sys,AtlasState(r));
      obj.robot = r;
      obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),'manip');
      obj = addLCMTransition(obj,'ATLAS_COMMAND_UNSAFE',drc.atlas_command_t(),'manip');  
      obj = addLCMTransition(obj,'START_MIT_STAND',drc.utime_t(),'stand');  
    end

    function msg = status_message(obj,t_sim,t_ctrl)
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


