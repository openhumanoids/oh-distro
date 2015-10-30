classdef LCMInputFromExternalForceBlock < MIMODrakeSystem
  
  properties
    lc;
    lcmonitor_external_force;
    force_magnitude=0;
    manip;
    force_torque = {}; % stores the force torque gotten from LCM Handlers, one for each ForceElement
    % each element of the cell should be a 6x1 array
  end
  
  methods
    function obj = LCMInputFromExternalForceBlock(r_complete, force_element)

      obj.manip = r_complete.getManipulator();
      obj.num_force_elements = length(obj.manip.force);
      obj.force_elements = obj.manip.force;

      output_frame = MultiCoordinateFrame.constructFrame(manip.force);
      frames = {};
      for i=1:obj.num_force_elements
        frames{i} = obj.force_elements{i}.constructFrame();
        obj.force_torque{i} = zeros(6,1);
      end

      output_frame = MultiCoordinateFrame.constructFrame(frames);
      input_frame = CoordinateFrame('empty', 0);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      obj.force_magnitude=0;
      

      % need to setup the LCM handlers . . . 

      
    end
    
    
    function varargout=mimoOutput(obj,t,x,varargin)
      varargout = {obj.force_torque};      
    end

  end
  
end
