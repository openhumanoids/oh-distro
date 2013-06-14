classdef CrawlingController < DRCController
    
  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end
    
  methods
    function obj = CrawlingController(name,r,options)
      typecheck(r,'Atlas');
    
        ctrl_data = SharedDataHandle(struct('qtraj',zeros(getNumDOF(r),1),...
              'qdtraj',zeros(getNumDOF(r),1),...
              'qddtraj',zeros(getNumDOF(r),1),...
              'support_times',[],...
              'supports',[],...
              'ignore_terrain',false));

        sys = PosVelFeedForwardBlock(r,ctrl_data,options);
        obj = obj@DRCController(name,sys,AtlasState(r));
        obj.robot = r;
        obj.controller_data = ctrl_data;

        obj = setTimedTransition(obj,inf,name,false);
        obj = addLCMTransition(obj,'WALKING_PLAN',drc.walking_plan_t(),'crawling');  % for crawling
    end
        
    function msg = status_message(obj,t_sim,t_ctrl)
      msg = drc.controller_status_t();
      msg.utime = t_sim * 1000000;
      msg.state = msg.CRAWLING;
      msg.controller_utime = t_ctrl * 1000000;
      msg.V = 0;
      msg.Vdot = 0;
    end        

    function obj = initialize(obj,data)

      % TODO: put some error handling in here
      tmp_fname = ['tmp_w_', num2str(feature('getpid')), '.mat'];

      msg_data = data.WALKING_PLAN;
      % do we have to save to file to convert a byte stream to a
      % matlab binary?

      support_times = msg_data.support_times;
      obj.controller_data.setField('support_times',support_times);

      fid = fopen(tmp_fname,'w');
      fwrite(fid,typecast(msg_data.supports,'uint8'),'uint8');
      fclose(fid);
      matdata = load(tmp_fname);
      if iscell(matdata.supports) 
        warning('somebody sent me a cell array of supports.  don''t do that anymore!');
        matdata.supports = [matdata.supports{:}];
      end
      obj.controller_data.setField('supports',matdata.supports);

      fid = fopen(tmp_fname,'w');
      fwrite(fid,typecast(msg_data.qtraj,'uint8'),'uint8');
      fclose(fid);
      matdata = load(tmp_fname);
      qdtraj = fnder(matdata.qtraj,1);
      qddtraj = fnder(qdtraj,1);
      obj.controller_data.setField('qtraj',matdata.qtraj);
      obj.controller_data.setField('qdtraj',qdtraj);
      obj.controller_data.setField('qddtraj',qddtraj);

      delete(tmp_fname);

      % obj = setDuration(obj,tspan_end,false); % set the controller timeout


    end
  end
    
end
