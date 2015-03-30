classdef LCMInputFromForceTorqueBlock < MIMODrakeSystem
  
  properties
    lc;
    lcmonitor_cmd; %LCM monitors
    r;
    
    last_wrench;
    
  end
  
  methods
    function obj = LCMInputFromForceTorqueBlock(r, options)
      typecheck(r,'Atlas');
      
      if nargin<3
        options = struct();
      end

      % Generate AtlasInput as out (we'll do translation manually)
      output_frame = r.getInputFrame.getFrameByName('three_dof_forceInput');
      
      % We need no inputs
      input_frame = CoordinateFrame('empty', 0, 'e');
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      obj.r = r;
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmonitor_cmd = drake.util.MessageMonitor(drake.lcmt_force_torque,'timestamp');
      obj.lc.subscribe('SIM_WRENCH_INPUT',obj.lcmonitor_cmd);
      
      obj.last_wrench = SharedDataHandle([0;0;0]);
    end
    
    function varargout=mimoOutput(obj,~,~,~)
      
      % see if we have a new message (new command state)
      data = obj.lcmonitor_cmd.getMessage();
      if (~isempty(data))
        data = drake.lcmt_force_torque(data);
        obj.last_wrench.setData([data.fx; data.fy; data.fz]);
      end
      
      wrench = obj.last_wrench.getData();
      
      varargout = {wrench};
      
    end
  end
  
end
