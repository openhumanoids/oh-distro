classdef LCMInputFromExternalForceBlock < MIMODrakeSystem
  
  properties
    lc;
    lcmonitor_external_force;
    force_magnitude=0;
  end
  
  methods
    function obj = LCMInputFromExternalForceBlock(r_complete, force_element)

      % Generate AtlasInput as out (we'll do translation manually)
      output_frame = r_complete.getInputFrame.frame{2};
      
      % We'll need atlas state as input
      input_frame = CoordinateFrame('empty', 0);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      obj.force_magnitude=0;
      
      
      % obj.lc = lcm.lcm.LCM.getSingleton();
      % obj.lcmonitor_external_force = drake.util.MessageMonitor(drc.atlas_command_t,'utime');
      % obj.lc.subscribe('EXTERNAL_FORCE',obj.lcmonitor_neck);

      
    end
    
    
    function varargout=mimoOutput(obj,t,x,varargin)
      varargout = {obj.force_magnitude};      
    end

  end
  
end
