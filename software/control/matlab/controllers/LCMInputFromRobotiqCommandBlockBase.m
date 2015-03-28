classdef LCMInputFromRobotiqCommandBlockBase < MIMODrakeSystem
  
  properties
    lc; % LCM
    lcmonitor; %LCM monitor
    lcmtype_constructor;
    lcmtype;
    coordinate_names;
    timestamp_name;
    dim;
    % Atlas for referencing hand input:
    r;
    %target finger positions
    target_positions = [0;0;0];
  end
  
  methods
    function obj = LCMInputFromRobotiqCommandBlockBase(r, handedness, options)
      typecheck(r,{'Atlas', 'IRB140'});

      if (~strcmp(handedness, 'left') && ~strcmp(handedness, 'right'))
        error('Handedness must be "left" or "right"');
      end


      output_frame = r.getInputFrame().getFrameByName([handedness, '_atlasFrames.HandInput']);
      input_frame = r.getOutputFrame().getFrameByName([handedness, '_atlasFrames.HandState']);

      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      obj.r = r;
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmonitor = drake.util.MessageMonitor(robotiqhand.command_t,'utime');
      if (strcmp(handedness, 'right'))
        obj.lc.subscribe('ROBOTIQ_RIGHT_COMMAND',obj.lcmonitor);
      else
        obj.lc.subscribe('ROBOTIQ_LEFT_COMMAND',obj.lcmonitor);
      end

      lcmtype = robotiqhand.command_t;
      lcmtype = lcmtype.getClass();
      names={};
      f = lcmtype.getFields;
      for i=1:length(f)
        fname = char(f(i).getName());
        if strncmp(fname,'LCM_FINGERPRINT',15), continue; end
        if strcmp(fname,'utime')
          if ~strcmp(f(i).getGenericType.getName,'long')
            error('by convention, the timestamp field should be type int64_t');
          end
          obj.timestamp_name = 'utime';
          continue;
        end
        names{end+1}=fname;
      end
      obj.lcmtype = lcmtype;
      obj.coordinate_names = names;
      obj.dim = length(names);
      constructors = lcmtype.getConstructors();
      for i=1:length(constructors)
        f = constructors(i).getParameterTypes;
        if ~isempty(f) && strncmp('[B',char(f(1).getName),2)
          obj.lcmtype_constructor = constructors(i);
        end
      end
    end
    
    function obj = grabLCMMessage(obj)
      data = obj.lcmonitor.getMessage();
      % If we haven't received a command, apply outward force
      if (~isempty(data))
        cmd = obj.decode(data);
        % Use this to set our goal state
        if cmd.mode==0
          obj.target_positions(:) = (cmd.position/255);
        else
          % Don't know how to handle this yet
          fprintf('Nonzero hand control mode\n')
        end
      end
    end

    function x=decode(obj, data)
      msg = obj.lcmtype_constructor.newInstance(data);
      for i=1:obj.dim
        eval(['x.',obj.coordinate_names{i},' = msg.',CoordinateFrame.stripSpecialChars(obj.coordinate_names{i}),';']);
      end
      eval(['t = msg.', obj.timestamp_name, '/1000;']);
    end
    
    function varargout=mimoOutput(obj,t,~,hand_state)
      varargout = {};
    end
  end
  
end
