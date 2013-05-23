classdef WalkingController < DRCController
  
  methods
    function obj = WalkingController(name,r)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct(...
        'A',[zeros(2),eye(2); zeros(2,4)],...
        'B',[zeros(2); eye(2)],...
        'C',[eye(2),zeros(2)],...
        'D',[],...
        'Qy',eye(2),...
        'S',[],...
        's1',[],...
        'x0',zeros(4,1),...
        'u0',zeros(2,1),...
        'y0',zeros(2,1),...
        'comtraj',[],...
        'lfoottraj',[],...
        'rfoottraj',[],...
        'supptraj',[],...
        'qtraj',zeros(getNumDOF(r),1)));

      % instantiate QP controller
      options.slack_limit = 30.0;
      options.w = 0.01;
      options.lcm_foot_contacts = true;
      options.full_body_opt = false; % if false, doesn't include arms/neck in QP solve (faster)
      nu=getNumInputs(r);
      options.R = 1e-12*eye(nu);
      input_names = r.getInputFrame.coordinates;
      ankle_idx = ~cellfun(@isempty,strfind(input_names,'lax')) | ~cellfun(@isempty,strfind(input_names,'uay'));
      ankle_idx = find(ankle_idx);
      options.R(ankle_idx,ankle_idx) = 10*options.R(ankle_idx,ankle_idx); % soft ankles
      options.use_mex = true;
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
      
      % cascade footstep replanner 
      fs = FootstepReplanner(r,ctrl_data);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 2;
      ins(2).input = 2;
      outs(1).system = 2;
      outs(1).output = 1;
      connection.from_output = 1;
      connection.to_input = 1;
      sys = mimoCascade(fs,sys,connection,ins,outs);
      
      % cascade neck pitch control block
      neck = NeckControlBlock(r,ctrl_data);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 1;
      ins(2).input = 2;
      ins(3).system = 2;
      ins(3).input = 2;
      outs(1).system = 2;
      outs(1).output = 1;
      connection.from_output = 1;
      connection.to_input = 1;
      sys = mimoCascade(neck,sys,connection,ins,outs);
      
      obj = obj@DRCController(name,sys);

      obj.controller_data = ctrl_data;
      obj = setTimedTransition(obj,100,'standing',false); % default timeout
      
   end
    
    function obj = initialize(obj,data)
            
      msg_data = getfield(data,'WALKING_PLAN');
      % do we have to save to file to convert a byte stream to a
      % matlab binary?
      fid = fopen('tmp_w.mat','w');
      fwrite(fid,typecast(msg_data.S,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
%       obj.controller_data.setField('S',matdata.S);
      obj.controller_data.setField('S',matdata.S.eval(0)); % S is always constant

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
      fwrite(fid,typecast(msg_data.zmptraj,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      obj.controller_data.setField('x0',[matdata.zmptraj.eval(matdata.zmptraj.tspan(2));0;0]);
      obj.controller_data.setField('y0',matdata.zmptraj);

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
