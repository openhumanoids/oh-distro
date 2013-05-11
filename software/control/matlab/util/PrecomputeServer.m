classdef PrecomputeServer

  properties (SetAccess=protected,GetAccess=protected)
    robot;
    monitors; 
    channels;
    precomp_funs;
    lc;
  end

  methods
    function obj = PrecomputeServer(r)
      typecheck(r,'Atlas');
      obj.robot = r;
      obj.monitors={}; 
      obj.channels={};
      obj.precomp_funs={};
      obj.lc = lcm.lcm.LCM.getSingleton();
    end
    
    function obj = addPrecomputeNode(obj,channel,precomp_fun)
      typecheck(channel,'char');
      typecheck(precomp_fun,'function_handle');
      
      if any(strcmp(channel,obj.channels))
        error(['PrecomputeServer:addPrecomputeNode: Channel name ', channel,' already exists.']);
      end

      lcmtype = drc.precompute_request_t();
      mon = drake.util.MessageMonitor(lcmtype,'utime');
      obj.lc.subscribe(channel,mon);
      
      n = length(obj.monitors)+1;
      obj.channels{n} = channel;
      obj.monitors{n} = mon;
      obj.precomp_funs{n} = precomp_fun;
    end
    
    function checkPrecomputeRequests(obj)
      for i=1:length(obj.monitors)
        req = obj.monitors{i}.getNextMessage(1);
        if ~isempty(req)
          disp('received precompute request');

          f = obj.precomp_funs{i};
          req_msg = drc.precompute_request_t(req);
          resp_data = f(obj.robot,req_msg); 
          resp = drc.precompute_request_t();
          resp.utime = 0; % put something here
          resp.robot_name = req_msg.robot_name;
          resp.response_channel = obj.channels{i};
        	resp.precompute_type = req_msg.precompute_type;
          
          save('prec_r.mat','resp_data');
          fid = fopen('prec_r.mat','r');
          resp.matdata = fread(fid,inf,'*uint8');
          fclose(fid);
          resp.n_bytes = length(resp.matdata); 
          disp('publishing precompute response');
          obj.lc.publish(req_msg.response_channel, resp);
        end
      end
    end
    
    function run(obj)
      while true
        checkPrecomputeRequests(obj);
        hb = drc.utime_t();
        hb.utime = 0; % put something useful here?
        obj.lc.publish('PRECOMP_SERVER_HEARTBEAT',hb);
        
        pause(0.025);
      end
    end
  end
  
end
