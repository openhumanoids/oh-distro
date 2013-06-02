classdef WalkingController < DRCController
  
  methods
    function obj = WalkingController(name,r,options)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct(...
        'A',[zeros(2),eye(2); zeros(2,4)],...
        'B',[zeros(2); eye(2)],...
        'C',[eye(2),zeros(2)],...
        'D',[],...
        'Qy',eye(2),...
        'S',[],...
        's1',[],...
        's2',[],...
        'x0',zeros(4,1),...
        'u0',zeros(2,1),...
        'y0',zeros(2,1),...
        'comtraj',[],...
        'link_constraints',[],...
        'z_drift',0,... % we subtract this value from the output of each foot trajectory to compensate for drift
        'support_times',[],...
        'supports',[],...
        'mu',1.0,...
        'qtraj',zeros(getNumDOF(r),1),...
        'V',0,... % cost to go used in controller status message
        'Vdot',0)); % time derivative of cost to go used in controller status message

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
      if(~isfield(options,'use_mex')) options.use_mex = false; end
      if(~isfield(options,'debug')) options.debug = false; end

      options.lcm_foot_contacts = true;
      qp = QPController(r,ctrl_data,options);

      % cascade walking PD controller 
      pd = WalkingPDBlock(r,ctrl_data);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 1;
      ins(2).input = 2;
      ins(3).system = 2;
      ins(3).input = 2;
      outs(1).system = 2;
      outs(1).output = 1;
      sys = mimoCascade(pd,qp,[],ins,outs);
      
      % cascade neck pitch control block
      neck = NeckControlBlock(r,ctrl_data);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 1;
      ins(2).input = 2;
      ins(3).system = 2;
      ins(3).input = 3;
      outs(1).system = 2;
      outs(1).output = 1;
      connection(1).from_output = 1;
      connection(1).to_input = 1;
      connection(2).from_output = 2;
      connection(2).to_input = 2;
      sys = mimoCascade(neck,sys,connection,ins,outs);
      clear connection ins outs;
      
      % cascade footstep replanner 
      fs = FootstepReplanner(r,ctrl_data);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 2;
      ins(2).input = 1;
      ins(3).system = 2;
      ins(3).input = 3;
      outs(1).system = 2;
      outs(1).output = 1;
      connection.from_output = 1;
      connection.to_input = 2;
      sys = mimoCascade(fs,sys,connection,ins,outs);
      
      obj = obj@DRCController(name,sys);

      obj.controller_data = ctrl_data;
      obj = setTimedTransition(obj,100,'standing',false); % default timeout
      
      obj = addLCMTransition(obj,'BRACE_FOR_FALL',drc.utime_t(),'bracing');
    end
    
    function send_status(obj,t_sim,t_ctrl)
        msg = drc.controller_status_t();
        msg.utime = t_sim * 1000000;
        msg.state = msg.WALKING;
        msg.controller_utime = t_ctrl * 1000000;
        msg.V = obj.controller_data.getField('V');
        msg.Vdot = obj.controller_data.getField('Vdot');
        obj.lc.publish('CONTROLLER_STATUS',msg);
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
      fwrite(fid,typecast(msg_data.s2,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      obj.controller_data.setField('s2',matdata.s2);
      
      support_times = msg_data.support_times;
      obj.controller_data.setField('support_times',support_times);

      fid = fopen('tmp_w.mat','w');
      fwrite(fid,typecast(msg_data.supports,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      obj.controller_data.setField('supports',matdata.supports);

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
      fwrite(fid,typecast(msg_data.link_constraints,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      obj.controller_data.setField('link_constraints',matdata.link_constraints);
      if ~isempty(matdata.link_constraints(1).traj)
        tspan_end = matdata.link_constraints(1).traj.tspan(end);
      else
        tspan_end = matdata.link_constraints(1).min.tspan(end);
      end
      
      fid = fopen('tmp_w.mat','w');
      fwrite(fid,typecast(msg_data.qnom,'uint8'),'uint8');
      fclose(fid);
      matdata = load('tmp_w.mat');
      obj.controller_data.setField('qtraj',matdata.qnom);

      obj.controller_data.setField('mu',msg_data.mu);

      obj = setDuration(obj,tspan_end,false); % set the controller timeout
    end
  end  
end
