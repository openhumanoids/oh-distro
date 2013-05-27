classdef DRCPlanner < handle

  properties (SetAccess=private,GetAccess=private)
    monitors;  % array of MessageMonitors
    last_msg_utimes; % the most recent utime for each monitor which was handled in updateData
    coders;  % array of lcmCoders
    constructors={};  % array of lcm type constructors
    required=[];  % boolean array saying whether each message monitor is required
    updatable=[]; % boolean array saying whether each input should be updated during the plan
    can_trigger=[]; % boolean array saying whether each input can trigger a new plan
    name={};
    %  plan(msg,data)
  end

  methods 
    plan(obj,data)  % planners need to implement this
    %  where request_msg is the plan_request_lcmtype and data is a structure 
    %  with fields containing the input names, populated with the
    %  corresponding lcmtypes
    %  use updateData() from inside plan to do interactive planning
  end
  
  methods
    function obj = DRCPlanner()
    end
    
    function obj = addInput(obj,name,channel,lcmtype_or_lcmcoder,required,updatable,can_trigger)
      if (nargin<5) required = false; end
      if (nargin<6) updatable = true; end
      if (nargin<7) can_trigger = false; end
      typecheck(name,'char');
      typecheck(channel,'char');
      n = length(obj.monitors)+1;
      if typecheck(lcmtype_or_lcmcoder,'LCMCoder')
        obj.coders{n} = lcmtype_or_lcmcoder;
        lcmtype = obj.coders{n}.encode(0,zeros(obj.coders{n}.dim(),1));
      else
        obj.coders{n} = [];
        [lcmtype,obj.constructors{n}]=DRCPlanner.parseLCMType(lcmtype_or_lcmcoder);
      end
      mon = drake.util.MessageMonitor(lcmtype,'utime');
      % mon = lcm.lcm.MessageAggregator();
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe(channel,mon);
      obj.monitors{n} = mon;
      obj.required(n) = required;
      obj.updatable(n) = updatable;
      obj.can_trigger(n) = can_trigger;
      obj.last_msg_utimes(n) = -1;
      obj.name{n} = name;
    end

    function [data,changed,changelist] = updateData(obj,data)
      % @retval data is the updated data
      % @retval changed is a boolean specifying whether ANYTHING changed
      % @retval changelist is a structure with the same fields as data
      % which has boolean entries specifying which elements of data
      % changed.  (e.g. data.x0 = false)
      changed=false; changelist=struct();
      for i=1:length(obj.monitors)
        changelist = setfield(changelist,obj.name{i},false);
        % if (obj.updatable(i) && getLastTimestamp(obj.monitors{i})>=data.utime)
        % if obj.updatable(i)
        if obj.updatable(i) && (getLastTimestamp(obj.monitors{i}) >= obj.last_msg_utimes(i))
        % if (obj.updatable(i) ...
        %     && (getLastTimestamp(obj.monitors{i}) > obj.last_msg_utimes(i) ...
        %         || obj.always_process(i)))
          % data.utime = max(data.utime,getLastTimestamp(obj.monitors{i}));
          obj.last_msg_utimes(i) = getLastTimestamp(obj.monitors{i});
          % d = getNextMessage(obj.monitors{i}, 0);
          d = getNextMessage(obj.monitors{i}, 0);
          % if obj.always_process(i)
          %   d = getNextMessage(obj.monitors{i}, 1);
          % else
          %   d = getMessage(obj.monitors{i});
          % end
          if ~isempty(d)
            if isempty(obj.coders{i})
              % data = setfield(data,obj.name{i},obj.lcmtype_constructor.newInstance(d));
              data = setfield(data,obj.name{i},obj.constructors{i}.newInstance(d));
            else
              data = setfield(data,obj.name{i},obj.coders{i}.decode(d));
            end
            changelist = setfield(changelist,obj.name{i},true);
            changed = true;
          end
        end
      end
    end
    
    function run(obj)
      % waits for plan requests and, upon receipt, checks to make we've
      % received all of the required bits and then calls the plan method
      % note: this method will never return (hit ctrl-c to cancel)
      
      tic;
      data = struct('utime', 0);
      while (1)
        [data, changed, changelist] = obj.updateData(data);
        if changed
          triggered = false;
          has_required = true;
          for i = 1:length(obj.name)
            n = obj.name{i};
            if ~isfield(data, n) && obj.required(i)
              has_required = false;
            end
            if isfield(changelist, n) && changelist.(n) && obj.can_trigger(i)
              triggered = true;
            end
          end
          if has_required && triggered
            disp('planning now')
            plan(obj,data)
            tic
          elseif triggered && ~has_required
            disp('missing required inputs.  this plan request will be ignored.');
          end
        end
        pause(1);
        fprintf(1, 'waiting... (t=%f)\n', toc);
      end
    end

    function runParallel(obj,num_jobs)
      % todo: implement this 
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