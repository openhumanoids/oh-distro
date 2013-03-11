classdef DRCPlanner

  properties (SetAccess=private,GetAccess=private)
    request_monitor;
    request_constructor;
    request_coder;
    monitors;  % array of MessageMonitors
    coders;  % array of lcmCoders
    constructors=[];  % array of lcm type constructors
    required=[];  % boolean array saying whether each message monitor is required
    updatable=[]; % boolean array saying whether each input should be updated during the plan
    name={};
    %  plan(msg,data)
  end

  methods 
    plan(obj,request_msg,data)  % planners need to implement this
    %  where request_msg is the plan_request_lcmtype and data is a structure 
    %  with fields containing the input names, populated with the
    %  corresponding lcmtypes
    %  use updateData() from inside plan to do interactive planning
  end
  
  methods
    function obj = DRCPlanner(plan_request_channel,plan_request_lcmtype_or_lcmcoder)
      typecheck(plan_request_channel,'char');
      if typecheck(plan_request_lcmtype_or_lcmcoder,'LCMCoder')
        obj.request_coder = plan_request_lcmtype_or_lcmcoder;
        lcmtype = obj.request_coder.encode(0,zeros(obj.request_coder.dim(),1));
      else
        [lcmtype,obj.request_constructor]=DRCPlanner.parseLCMType(plan_request_lcmtype_or_lcmcoder);
      end
      obj.request_monitor = drake.util.MessageMonitor(lcmtype,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe(plan_request_channel,obj.request_monitor);
    end
    
    function obj = addInput(obj,name,channel,lcmtype_or_lcmcoder,required,updatable)
      typecheck(name,'char');
      typecheck(channel,'char');
      n = length(obj.monitors)+1;
      if typecheck(lcmtype_or_lcmcoder,'LCMCoder')
        obj.coders{n} = lcmtype_or_lcmcoder;
        lcmtype = obj.coders{n}.encode(0,zeros(obj.coders{n}.dim(),1));
      else
        [lcmtype,obj.constructors(n)]=DRCPlanner.parseLCMType(lcmtype_or_lcmcoder);
      end
      if (nargin<4) required = false; end
      if (nargin<5) updatable = true; end
      mon = drake.util.MessageMonitor(lcmtype,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe(channel,mon);
      obj.monitors{n} = mon;
      obj.required(n) = required;
      obj.updatable(n) = updatable;
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
        if (obj.updatable(i) && getLastTimestamp(obj.monitors{i})>data.utime)
          data.utime = max(data.utime,getLastTimestamp(obj.monitors{i}));
          d = getMessage(obj.monitors{i});
          if ~isempty(d)
            if isempty(obj.coders{i})
              data = setfield(data,obj.name{i},obj.lcmtype_constructor.newInstance(d));
            else
              data = setfield(data,obj.name{i},obj.coders{i}.decode(d));
            end
            changelist = setfield(changelist,obj.name{i},true);
          end
        end
      end
    end
    
    function run(obj)
      % waits for plan requests and, upon receipt, checks to make we've
      % received all of the required bits and then calls the plan method
      % note: this method will never return (hit ctrl-c to cancel)
      
      tic;
      while (1)
        fd = getNextMessage(obj.request_monitor,1000);
        if isempty(fd)
          fprintf(1,'waiting... (t=%f)\n',toc);
        else
          if isempty(obj.request_coder)
            msg = obj.lcmtype_constructor.newInstance(fd);
            utime=msg.utime;
          else
            [msg,tmsg] = obj.request_coder.decode(fd);
            utime = int64(tmsg*1000000);
          end
          data=struct('utime',utime);
          has_required = true;
          for i=1:length(obj.monitors)
            d = getMessage(obj.monitors{i});
            if ~isempty(d)
              if isempty(obj.coders{i})
                data = setfield(data,obj.name{i},obj.lcmtype_constructor.newInstance(d));
              else
                data = setfield(data,obj.name{i},obj.coders{i}.decode(d));
              end
            elseif obj.required(i)
              fprintf(1,'required input %s has not been received yet.\n',obj.name{i});
              has_required = false;
            end
          end
          if has_required
            plan(obj,msg,data);
          else
            disp('missing required inputs.  this plan request will be ignored.');
          end
          tic;
        end
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