classdef LCMInputFromExternalForceBlock < MIMODrakeSystem
  
  properties
    lc;
    lcm_monitor = {};
    lcmonitor_external_force;
    force_magnitude=0;
    num_force_elements = 0;
    force_elements = {};
    manip;
    body_id = [];
    stopwatch = {};
    body_name = {};
    last_msg_time;
    force_torque = {}; % stores the force torque gotten from LCM Handlers, one for each ForceElement
    % each element of the cell should be a 6x1 array
    timeout = 1;

  end
  
  methods
    function obj = LCMInputFromExternalForceBlock(r_complete)

      
      manip = r_complete.getManipulator();
      num_force_elements = length(manip.force);
      force_elements = manip.force;

      frames = {};
      for i=1:num_force_elements
        frames{i} = force_elements{i}.constructFrame();
        % obj.force_torque{i} = zeros(6,1); % initialize
        % obj.body_id(i) = obj.force_elements{i}.body_id;
      end

      output_frame = MultiCoordinateFrame.constructFrame(frames);
      input_frame = CoordinateFrame('empty', 0);
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      obj.force_magnitude=0;

      obj.num_force_elements = num_force_elements;
      obj.force_elements = force_elements;

      for i=1:obj.num_force_elements
        obj.body_id(i) = obj.force_elements{i}.body_id;
        obj.body_name{i} = r_complete.getLinkName(obj.body_id(i));
        obj.force_torque{i} = zeros(6,1); % initialize
        obj.stopwatch{i} = tic;
      end

      

      % need to setup the LCM handlers . . . 
      obj.lc = lcm.lcm.LCM.getSingleton();
      for i=1:obj.num_force_elements
        obj.lcm_monitor{i} = drake.util.MessageMonitor(drake.lcmt_external_force_torque, 'timestamp');
        channel_name = strcat('EXTERNAL_FORCE_TORQUE_', int2str(obj.body_id(i)));
        obj.lc.subscribe(channel_name, obj.lcm_monitor{i});
      end
      

    end
    
    
    function varargout=mimoOutput(obj,t,x,varargin)
      varargout = {};
      for i=1:obj.num_force_elements

        if (obj.lcm_monitor{i}.hasNewMessage())
          obj.stopwatch{i} = tic;
          data = obj.lcm_monitor{i}.getMessage();
          if (~isempty(data))
            if strcmp(data.body_name, obj.body_name{i})
              error('body name for this force element does not match the one in the message')
            end
          end
          data = drake.lcmt_external_force_torque(data);
          obj.force_torque{i} = [data.tx; data.ty; data.tz; data.fx; data.fy; data.fz];
        end

        % if we received our last new message within timeout, then publish that through to the sim
        % otherwise set the force-torque to zero
        if (toc(obj.stopwatch{i}) < obj.timeout)
          varargout{i} = obj.force_torque{i};
        else
          varargout{i} = zeros(6,1);
        end
      end

    end

  end
  
end
