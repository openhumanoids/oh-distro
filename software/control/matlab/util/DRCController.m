classdef DRCController

  properties (SetAccess=protected,GetAccess=public)
    name=''; % controller name
  end  
  
  properties (SetAccess=protected,GetAccess=protected)
    controller; % drake system
    controller_data; % optional shared data handle reference
    
    controller_input_frames; % lcm frames w/coders for controller inputs
    n_input_frames;
    controller_output_frame;
    
    transition_coders; % lcm coders for transition events
    transition_monitors; % message monitors for transition events
    transition_targets; % list of names of controllers to transition to
    transition_channels; 

    precompute_triggers; % list of function handles, active=f(controller_input,t) 
    precompute_active; % list of bools, whether trigger functions are currently active 
    
    precompute_response_channels;
    precompute_response_monitors;
    precompute_response_targets; % struct that maps controller names to received messages
    
    constructors; % cell array of lcm type constructors (function handles)
    t_final = inf; % controller time limit
    timed_transition; % name of the controller to transition to when t>=t_final
    absolute_time; % bool: whether t_final is absolute or relative to start time

    lc;
  end

  methods (Abstract)
    initialize(obj,data); % controllers need to implement this
    %  in the event of a lcm transition, data contains a struct that maps channel names to the 
    %  decoded lcm message data. for timed transitions, it maps input frame names to the latest data
    msg = status_message(obj,t_sim,t_ctrl); % each controller should populate a drc_controller_status_t message
    %  and send it back to the base station
  end

  methods
    function obj = DRCController(name,sys)
      typecheck(name,'char');
      if ~(isa(sys,'DrakeSystem') || isa(sys,'SimulinkModel'))
        error('DRCController::Argument sys should be a DrakeSystem or SimulinkModel');
      end
      if ~isDT(sys)
        error('DRCController: only supports discrete time systems');
      end
        
      obj.name = name;
      obj.controller = sys;
      
      if typecheck(sys.getInputFrame,'MultiCoordinateFrame')
        obj.controller_input_frames = sys.getInputFrame.frame;
        obj.n_input_frames = length(obj.controller_input_frames);
      else
        obj.controller_input_frames{1} = obj.controller.getInputFrame();
        obj.n_input_frames = 1; 
      end
      obj.controller_output_frame = obj.controller.getOutputFrame();
      
      for i=1:obj.n_input_frames
        obj.controller_input_frames{i}.subscribe(defaultChannel(obj.controller_input_frames{i}));
      end
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      
      % in order to support backup mode:
      typecheck(obj.controller_output_frame,'LCMCoordinateFrameWCoder'); % could be more general, but this should get us started
      obj.controller_output_frame.subscribe(defaultChannel(obj.controller_output_frame));
    end
    
    function obj = setTimedTransition(obj,t_final,transition_to_controller,absolute_time)
      typecheck(t_final,'double');
      typecheck(transition_to_controller,'char');
      if nargin > 3
        typecheck(absolute_time,'logical');
      else
        absolute_time = false;
      end

      obj.t_final = t_final;
      obj.timed_transition = transition_to_controller;
      obj.absolute_time = absolute_time;
    end

    function obj = setDuration(obj,t_final,absolute_time)
      typecheck(t_final,'double');
      if nargin > 2
        typecheck(absolute_time,'logical');
      else
        absolute_time = false;
      end

      obj.t_final = t_final;
      obj.absolute_time = absolute_time;
    end

    function tf = getDuration(obj)
      tf = obj.t_final;
    end
    
    function obj = addLCMTransition(obj,channel,lcmtype_or_lcmcoder,transition_to_controller)
      typecheck(channel,'char');
      typecheck(transition_to_controller,'char');

      n = length(obj.transition_monitors)+1;
      if typecheck(lcmtype_or_lcmcoder,'LCMCoder')
        obj.transition_coders{n} = lcmtype_or_lcmcoder;
        lcmtype = obj.transition_coders{n}.encode(0,zeros(obj.transition_coders{n}.dim(),1));
      else
        obj.transition_coders{n} = [];
        [lcmtype,obj.constructors{n}]=DRCController.parseLCMType(lcmtype_or_lcmcoder);
      end
      
      mon = drake.util.MessageMonitor(lcmtype,'utime');
      obj.lc.subscribe(channel,mon);
      
      obj.transition_channels{n} = channel;
      obj.transition_monitors{n} = mon;
      obj.transition_targets{n} = transition_to_controller;
    end

    function obj = addPrecomputeResponseHandler(obj,response_channel,transition_to_controller)
      typecheck(response_channel,'char');
      typecheck(transition_to_controller,'char');

      lcmtype = drc.precompute_request_t;
      mon = drake.util.MessageMonitor(lcmtype,'utime');
      obj.lc.subscribe(response_channel,mon);
      
      n = length(obj.precompute_response_monitors)+1;
      obj.precompute_response_channels{n} = response_channel;
      obj.precompute_response_monitors{n} = mon;
      obj.precompute_response_targets.(transition_to_controller) = [];
    end
    
    function obj = addPrecomputeTrigger(obj,trigger_function_handle)
      typecheck(trigger_function_handle,'function_handle');
      n = length(obj.precompute_triggers)+1;
      obj.precompute_triggers{n} = trigger_function_handle;
      obj.precompute_active{n} = true;
    end
    
    function [transition,data] = checkLCMTransitions(obj)
      data = struct();
      transition=false;
      
      for i=1:length(obj.transition_monitors)
        d = obj.transition_monitors{i}.getNextMessage(0);
        if ~isempty(d)
          if isempty(obj.transition_coders{i})
            data.(obj.transition_targets{i}) = struct(obj.transition_channels{i},obj.constructors{i}.newInstance(d));
          else
            data.(obj.transition_targets{i}) = struct(obj.transition_channels{i},obj.coders{i}.decode(d));
          end
          transition = true;
        end
      end
    end
    
    function obj=checkPrecomputeResponses(obj)
      if ~isempty(obj.precompute_response_monitors)
        fn = fieldnames(obj.precompute_response_targets);
        for i=1:length(obj.precompute_response_monitors)
          d = obj.precompute_response_monitors{i}.getNextMessage(0);
          if ~isempty(d)
            disp(['received precompute response on ' obj.precompute_response_channels{i}]);
            msg = drc.precompute_request_t(d);
            fid = fopen('prec_w.mat','w');
            fwrite(fid,typecast(msg.matdata,'uint8'),'uint8');
            fclose(fid);
            matdata = load('prec_w.mat');
            obj.precompute_response_targets.(fn{i}) = matdata;
          end
        end
      end
    end
    
    function [data,backup_mode] = run(obj,backup_mode)
      % runs the controller and, upon receiving a message on a termination
      % channel or if t >= t_final, halts and returns a struct mapping the
      % name of the controller to take over to lcm message data (or halting
      % time in the case of a timed transition)
      % 
      % @param if backup_mode=true, the controller will continue to
      % listen for incoming messages and run update.  It will also
      % subscribe to the messages that it is supposed to publish, and if it
      % does not hear that message, then it will switch out of backup mode
      % and start publishing (e.g. assuming that the primary controller has
      % crashed).
      
      if (nargin<2) backup_mode = false; end
      
      % on startup, populate input frames with last received data
      data = struct();
      input_frame_data = cell(obj.n_input_frames,1);
      for i=1:obj.n_input_frames
          [x,~] = getMessage(obj.controller_input_frames{i});
          if ~isempty(x)
            % use previous message
           input_frame_data{i} = x;
          else
            input_frame_data{i} = zeros(obj.controller_input_frames{i}.dim,1);
          end
      end
      
      % clear lcm transition buffers
      for i=1:length(obj.transition_monitors)
        obj.transition_monitors{i}.getNextMessage(0);
      end
      
%       missed_frames = 0;
%       max_state_delay = 0;
%       ttprev = [];
      
      t_offset = -1;
      lcm_check_tic = tic;
      status_tic = tic;
%       precompute_tic = tic;
      num_x = getNumStates(obj.controller);
      if (num_x>0), x = getInitialState(obj.controller); else x=[]; end
      while (1)
%         tic;
        if (toc(lcm_check_tic) > 0.5) % check periodically
          %obj=checkPrecomputeResponses(obj); % DISABLED
          % check termination conditions and break if any are true        
          [transition,data] = checkLCMTransitions(obj);
          lcm_check_tic = tic;
          if transition 
            % DISABLED PRECOMP STUFF
%             fn = fieldnames(data);
%             if isfield(obj.precompute_response_targets,fn{1})
%               d = obj.precompute_response_targets.(fn{1}); % take first transition if many
%               if ~isempty(d)
%                 data = struct('precomp',d); % pass precomputation message to next controller
%               end
%             end
 
            fn = fieldnames(data); % get channel names for transitions, take first one
            % append last input data
            tmpstruct = data.(fn{1});
            for i=1:obj.n_input_frames
              tmpstruct.(obj.controller_input_frames{i}.name) = input_frame_data{i};
            end
            data.(fn{1}) = tmpstruct;
            break;
          end
        end

        input_frame_time = -1*ones(obj.n_input_frames,1); % signify stale data with time -1
        checked_frames = inf*ones(obj.n_input_frames,1); % no name_hash can be inf
        % for each input subframe, get next message
        for i=1:obj.n_input_frames
          fr = obj.controller_input_frames{i};
          if any(fr.name_hash==checked_frames)
            continue;
          end
%          [x,tsim] = getNextMessage(fr,0);
          [x,tsim] = getNextMessage(fr,2); % in principle this should be 0 so we don't delay gets
          % on other input frames, but in our case most controllers just
          % have atlas state coming at high frequency, so we can use a
          % small wait to reduce the # of function calls
          if (~isempty(x))
%            fprintf(1,'i=%d, tsim=%f, %s\n',i,tsim,obj.controller_input_frames{i}.name);
            if (t_offset == -1)
              if obj.absolute_time
                t_offset = 0;
              else
                t_offset = tsim;
              end
            end
            t=tsim-t_offset;

            input_frame_data{i} = x;
            input_frame_time(i) = t;
            % copy data to other subframes
            checked_frames(i) = fr.name_hash;
            for j=1:obj.n_input_frames
              if (obj.controller_input_frames{j}.name_hash==fr.name_hash)
                input_frame_data{j} = x;
                input_frame_time(j) = t;
                markAsRead(obj.controller_input_frames{j}.monitor);
              end
            end
          end
        end

        % DISABLED BECAUSE WE'RE NOT USING IT
%         if toc(precompute_tic)>0.1
%           for i=1:length(obj.precompute_triggers)
%             if obj.precompute_active{i}
%               f=obj.precompute_triggers{i};
%               obj.precompute_active{i} = f(input_frame_data,input_frame_time);
%             end
%           end
%           precompute_tic=tic;
%         end
        
        tt = max(input_frame_time);
%         if isempty(ttprev)
%           ttprev=tt;
%         end
        
        if any(tt >= obj.t_final)
          % on timeout events, we pass back the latest input data unless
          % there is precomputed stuff available
          
%           d=[];
%           if isfield(obj.precompute_response_targets,obj.timed_transition)
%             d = obj.precompute_response_targets.(obj.timed_transition); 
%           end
%           if ~isempty(d)
%             input_data = struct('precomp',d); % pass precomputation message to next controller
%           else
            input_data = struct();
            for i=1:obj.n_input_frames
              input_data.(obj.controller_input_frames{i}.name) = input_frame_data{i};
            end
%           end
          data.(obj.timed_transition) = input_data;
          break;
        end
        
%         if all(input_frame_time == -1)
%           missed_frames = missed_frames +1;
%         end
        
        if any(input_frame_time >=0) % could also do 'all' here
%           max_state_delay = max(max_state_delay,tt-ttprev);
%           ttprev=tt;

          if backup_mode  % backup_mode logic
            % tt+t_offset is the timestamp of the message that I should be
            % sending.  if I haven't seen that message for 10msec, then come
            % out of backup_mode
            t_heartbeat = getLastTimestamp(obj.controller_output_frame) / 1e6;

            if t_heartbeat>0 && tt+t_offset - t_heartbeat > 0.1
              backup_mode = false;
              disp('HEARTBEAT MISSED.  TRANSITIONING OUT OF BACKUP MODE.');
            elseif toc(status_tic)>0.2
              % send the backup status message
              msg = status_message(obj,tt+t_offset,tt);
              obj.lc.publish('BACKUP_CONTROLLER_STATUS',msg);
              status_tic=tic;
            end
          else
            u = obj.controller.output(tt,x,vertcat(input_frame_data{:}));
            obj.controller_output_frame.publish(tt+t_offset,u,defaultChannel(obj.controller_output_frame));

            % publish controller status
            if toc(status_tic)>0.2
              msg = status_message(obj,tt+t_offset,tt);
              obj.lc.publish('CONTROLLER_STATUS',msg);
              status_tic=tic;
            end

            if num_x>0
              % note: for simulink models, this will call output again,
              % unless I pass in my new additional flag.  try that when we
              % get there.
              x = obj.controller.update(tt,x,vertcat(input_frame_data{:}));
            end
          end
  %         fprintf('Num missed frames: %d \n',missed_frames);
  %         fprintf('Max state delay: %2.3f sim secs \n',max_state_delay);
  %         toc
        end
      end
    end
  end
  
  methods (Static=true)
    function [lcmtype,constructor] = parseLCMType(lcmtype)
      if (ischar(lcmtype))
        lcmtype = eval(lcmtype);
      end
      if (~any(strcmp('getClass',methods(lcmtype))))
        error('lcmtype should be a valid java lcm object, or the string describing it');
      end
      lcmtypeClass=lcmtype.getClass();
      
      has_timestamp=false;
      names={};
      f = lcmtypeClass.getFields;
      for i=1:length(f)
        fname = char(f(i).getName());
        if strncmp(fname,'LCM_FINGERPRINT',15), continue; end
        if strcmp(fname,'utime'), 
          if ~strcmp(f(i).getGenericType.getName,'long')
            error('by convention, the timestamp field should be type int64_t');
          end
          has_timestamp=true; 
          continue; 
        end
      end
      if ~has_timestamp
        error('by convention, all lcm types should have a timestamp utime field of type int64_t');
      end
      
      constructors = lcmtypeClass.getConstructors();
      for i=1:length(constructors)
        f = constructors(i).getParameterTypes;
        if ~isempty(f) && strncmp('[B',char(f(1).getName),2)
          constructor = constructors(i);
        end
      end
      if isempty(constructor)
        error('didn''t find a constructor for this lcmtype');
      end
    end    
  end  
end
