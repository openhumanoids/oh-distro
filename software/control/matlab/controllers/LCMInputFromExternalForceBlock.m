classdef LCMInputFromExternalForceBlock < MIMODrakeSystem
  
  properties
    lc;
    lcm_monitor;
    num_force_elements = 0;
    force_elements = {};
    manip;
    body_id = [];
    shared_data_handle;
    body_name = {};
    last_msg_time;
    % shared_data_handle data is a struct with fields 'force_torque' and 'stopwatch'
    % 'force_torque' stores the FT data for each body,
    % 'stopwatch' stores the last time we geto a message with a force for that body
    timeout = 2;

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

      obj.num_force_elements = num_force_elements;
      obj.force_elements = force_elements;
      data = struct();
      data.force_torque = {};
      data.stopwatch = {};
      obj.shared_data_handle = SharedDataHandle(data);

      for i=1:obj.num_force_elements
        obj.body_id(i) = obj.force_elements{i}.body_id;
        obj.body_name{i} = obj.force_elements{i}.linkName;

        if obj.body_id(i) ~= r_complete.findLinkId(obj.body_name{i})
          errorString = strcat('linkname and body_id dont match for link ', linkName);
          error(errorString)
        end

        force_torque = obj.shared_data_handle.getField('force_torque');
        force_torque{i} = zeros(6,1); % initialize
        obj.shared_data_handle.setField('force_torque', force_torque)

        stopwatch = obj.shared_data_handle.getField('stopwatch');
        stopwatch{i} = tic;
        obj.shared_data_handle.setField('stopwatch', stopwatch);
      end

      

      % need to setup the LCM handlers . . . 
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcm_monitor = drake.util.MessageMonitor(drake.lcmt_external_force_torque, 'timestamp');
      obj.lc.subscribe('EXTERNAL_FORCE_TORQUE', obj.lcm_monitor)
    end
    
    
    function varargout=mimoOutput(obj,t,x,varargin)
      varargout = {};

      % only update the stored values if we have gotten a new message
      if (obj.lcm_monitor.hasNewMessage())
        data = obj.lcm_monitor.getMessage();
        if (~isempty(data))
          data = drake.lcmt_external_force_torque(data); % decode the message
          msg_num_forces = data.num_external_forces;

          for i=1:msg_num_forces
            msg_body_name = data.body_names(i);
            idx = find(strcmp(obj.body_name, msg_body_name));

            % if the body in the message isn't a force element for our robot, then just continue
            % to the next body specified in the message
            if isempty(idx)
              continue;
            end
            force_torque = obj.shared_data_handle.getField('force_torque');
            force_torque{idx} = [data.tx(i); data.ty(i); data.tz(i); data.fx(i); data.fy(i); ...
             data.fz(i)];

            obj.shared_data_handle.setField('force_torque', force_torque);

            stopwatch = obj.shared_data_handle.getField('stopwatch');
            stopwatch{idx} = tic;
            obj.shared_data_handle.setField('stopwatch', stopwatch);
            % strcat('resetting clock for idx ', num2str(idx))
          end

        end
      end

      % if we have received a msg for that body within the last obj.timeout seconds, then push that
      % force through to the simulation. Otherwise set that force to zero
      stopwatch = obj.shared_data_handle.getField('stopwatch');
      force_torque = obj.shared_data_handle.getField('force_torque');

      for j=1:obj.num_force_elements
        if (toc(stopwatch{j}) < obj.timeout)
          varargout{j} = force_torque{j};
        else
          force_torque{j} = zeros(6,1);
          varargout{j} = zeros(6,1);
        end
      end

      obj.shared_data_handle.setField('force_torque', force_torque);

    end

  end
  
end
