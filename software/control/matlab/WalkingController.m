classdef WalkingController < DRCController
  
  methods
    function obj = WalkingController(name,r)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct('A',[zeros(2),eye(2); zeros(2,4)],...
        'B',[zeros(2); eye(2)],'C',[eye(2),zeros(2)],'D',[],'Q',eye(2),...
        'S',[],'s1',[],'comtraj',[],'lfoottraj',[],'rfoottraj',[],'supptraj',[]));

      % instantiate QP controller
      options.slack_limit = 30.0;
      options.w = 0.1;
      options.R = 1e-12*eye(getNumInputs(r));
      qp = QPController(r,ctrl_data,options);

      % cascade PD qtraj controller 
      pd = WalkingPDController(r,ctrl_data);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 2;
      ins(2).input = 2;
      outs(1).system = 2;
      outs(1).output = 1;
      sys = mimoCascade(pd,qp,[],ins,outs);
      
      obj = obj@DRCController(name,sys);

      obj.controller_data = ctrl_data;
      obj = setTimedTransition(obj,100,'standing',false); % default timeout
      
   end
    
    function obj = initialize(obj,data)
            
      msg_data = getfield(data,'COMMITTED_WALKING_PLAN');
      % do we have to save to file to convert a byte stream to a
      % matlab binary?
      fid = fopen('tmp_w.mat','w');
      fwrite(fid,typecast(msg_data.htraj,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      htraj = matdata.htraj;
      
      fid = fopen('tmp_w.mat','w');
      fwrite(fid,typecast(msg_data.hddtraj,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      hddtraj = matdata.hddtraj;
      obj.controller_data.setField('D',-htraj/(hddtraj+9.81)*eye(2));
 
      fid = fopen('tmp_w.mat','w');
      fwrite(fid,typecast(msg_data.S,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      obj.controller_data.setField('S',matdata.S);

      fid = fopen('tmp_w.mat','w');
      fwrite(fid,typecast(msg_data.s1,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      obj.controller_data.setField('s1',matdata.s1);

      fid = fopen('tmp_w.mat','w');
      fwrite(fid,typecast(msg_data.supptraj,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      obj.controller_data.setField('supptraj',matdata.supptraj);

      fid = fopen('tmp_w.mat','w');
      fwrite(fid,typecast(msg_data.comtraj,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      obj.controller_data.setField('comtraj',matdata.comtraj);
      
      fid = fopen('tmp_w.mat','w');
      fwrite(fid,typecast(msg_data.lfoottraj,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      obj.controller_data.setField('lfoottraj',matdata.lfoottraj);

      fid = fopen('tmp_w.mat','w');
      fwrite(fid,typecast(msg_data.rfoottraj,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      obj.controller_data.setField('rfoottraj',matdata.rfoottraj);

      obj = setDuration(obj,matdata.rfoottraj.tspan(end),false); % set the controller timeout
    end
  end  
end
