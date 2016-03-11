classdef LCMInputFromIRB140AtlasCommandBlock < MIMODrakeSystem
  
  properties
    lc; % LCM
    lcmonitor; %LCM monitor
    lcmtype_constructor;
    coder;
    lcmtype;
    coordinate_names;
    timestamp_name;
    dim;
    
    last_receive_time;
    
    r;
    r_control;
    
    gravcomp;
  end
  
  methods
    function obj = LCMInputFromIRB140AtlasCommandBlock(r, r_control, options)
      typecheck(r,'IRB140');
      if ~isempty(r_control)
        typecheck(r_control, 'IRB140');
      else
        r_control = r;
      end
      
      if nargin<2
        options = struct();
      end

      % Generate AtlasInput as out (we'll do translation manually)
      output_frame = IRB140Input(r);
      input_frame = IRB140State(r_control);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      obj.r = r;
      obj.r_control = r_control;
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmonitor = drake.util.MessageMonitor(bot_core.atlas_command_t,'utime');
      obj.lc.subscribe('ATLAS_COMMAND',obj.lcmonitor);
      
      lcmtype = bot_core.atlas_command_t;
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
      
      % watchdog
      obj.last_receive_time = SharedDataHandle(-1);
      % gravcomp control
      obj.gravcomp = GravityCompensationBlock(r_control);
      
    end
    
    function [x,t]=decode(obj, data)
      msg = obj.lcmtype_constructor.newInstance(data);
      x=cell(obj.dim, 1);
      for i=1:obj.dim
        eval(['x{',num2str(i),'} = msg.',CoordinateFrame.stripSpecialChars(obj.coordinate_names{i}),';']);
        % fix string type
        if (isa(x{i}, 'java.lang.String[]'))
          x{i} = char(x{i});
        end
      end
      eval(['t = msg.', obj.timestamp_name, '/1000;']);
    end
    
    function varargout=mimoOutput(obj,t,~,varargin)
      % What needs to go out:
      efforts = zeros(obj.getNumOutputs, 1);
      irb140_state = varargin{1};
      
      % see if we have a new message (new command state)
      data = obj.lcmonitor.getMessage();
        
      % If we haven't received a command make our own
      cmd = [];
      if (~isempty(data))
        [cmd,tc] = obj.decode(data);
        if (obj.last_receive_time.getData == -1 || (~(tc>t*1000) && (t*1000 - tc < 15)))
          efforts = cmd{5};
          obj.last_receive_time.setData(tc)
        else
          cmd = [];
        end
      end
      
      if (isempty(cmd))
        % just try to do gravity comd and derivative kill
        K_D = 0.1*[1000; 500; 250;  10; 5; 1];
        efforts = -irb140_state(7:end).*K_D;
        y = obj.gravcomp.output(t, 0, irb140_state);
        efforts = efforts + y;
      end
        
      varargout = {efforts};
      %fprintf('\b\b\b\b\b\b\b%7.3f', t);
      
    end
  end
  
end
