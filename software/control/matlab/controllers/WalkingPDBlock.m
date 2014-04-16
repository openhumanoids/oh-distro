classdef WalkingPDBlock < MIMODrakeSystem
  % outputs a desired q_ddot (including floating dofs)
  properties
    nq;
    Kp;
    Kd;
    dt;
    controller_data; % pointer to shared data handle containing qtraj
    ikoptions;
    robot;
    max_nrm_err;
    contact_est_monitor;
    l_ank;
    r_ank;
    ik_qnom;
  end
  
  methods
    function obj = WalkingPDBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
            
      input_frame = MultiCoordinateFrame({AtlasCoordinates(r),r.getStateFrame});
      coords = AtlasCoordinates(r);
      obj = obj@MIMODrakeSystem(0,0,input_frame,coords,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,coords);

      obj.controller_data = controller_data;
      obj.nq = getNumDOF(r);

      if nargin<3
        options = struct();
      end
      
      if isfield(options,'Kp')
        typecheck(options.Kp,'double');
        sizecheck(options.Kp,[obj.nq 1]);
        obj.Kp = options.Kp;
        obj.Kp([1,2,6]) = 0; % ignore x,y,yaw
      else
        obj.Kp = 170.0*ones(obj.nq,1);
        obj.Kp([1,2,6]) = 0; % ignore x,y,yaw
      end        
        
      if isfield(options,'Kd')
        typecheck(options.Kd,'double');
        sizecheck(options.Kd,[obj.nq 1]);
        obj.Kd = options.Kd;
        obj.Kd([1,2,6]) = 0; % ignore x,y,yaw
      else
        obj.Kd = 19.0*ones(obj.nq,1);
        obj.Kd([1,2,6]) = 0; % ignore x,y,yaw
      end
            
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.001;
      end
      obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate

      state_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      obj.l_ank = find(~cellfun(@isempty,strfind(state_names,'l_leg_akx')) | ~cellfun(@isempty,strfind(state_names,'l_leg_aky')));
      obj.r_ank = find(~cellfun(@isempty,strfind(state_names,'r_leg_akx')) | ~cellfun(@isempty,strfind(state_names,'r_leg_aky')));
      
      if isfield(options,'q_nom')
        typecheck(options.q_nom,'double');
        sizecheck(options.q_nom,[obj.nq 1]);
        q_nom = options.q_nom;
        obj.controller_data.setField('qtraj',q_nom);
      else
        d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
        q_nom = d.xstar(1:obj.nq);
        obj.controller_data.setField('qtraj',q_nom);
      end
      
      if ~isfield(obj.controller_data,'trans_drift')
        obj.controller_data.setField('trans_drift',[0;0;0]);
      end
      
      % setup IK parameters
      cost = Point(r.getStateFrame,1);
      cost.base_x = 0;
      cost.base_y = 0;
      cost.base_z = 0;
      cost.base_roll = 1000;
      cost.base_pitch = 1000;
      cost.base_yaw = 0;
      cost.back_bkz = 1;
      cost.back_bky = 10;
      cost.back_bkx = 10;

      cost = double(cost);
      
%      arm_idx = find(~cellfun(@isempty,strfind(state_names,'arm')));
%      cost(arm_idx) = 0.1*ones(length(arm_idx),1);
      obj.ikoptions = struct();
      obj.ikoptions.Q = diag(cost(1:obj.nq));
      obj.ikoptions.q_nom = q_nom;

      % Prevent the knee from locking
      %[obj.ikoptions.jointLimitMin, obj.ikoptions.jointLimitMax] = r.getJointLimits();
      %joint_names = r.getStateFrame.coordinates(1:r.getNumDOF());
      %obj.ikoptions.jointLimitMin(~cellfun(@isempty,strfind(joint_names,'kny'))) = 0.6;

      obj.robot = r;
      obj.max_nrm_err = 1.5;
      
      obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe('FOOT_CONTACT_ESTIMATE',obj.contact_est_monitor);
      
      
    end
   
    function y=mimoOutput(obj,t,~,varargin)
      global infocount 
      if isempty(infocount)
        infocount = 0;
      end
      
      x = varargin{2};
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);

      obj.ikoptions.q_nom = varargin{1};
      cdata = obj.controller_data.data;
      
      Kp = obj.Kp;
      Kd = obj.Kd;
      
      % get foot contact state over LCM
      contact_data = obj.contact_est_monitor.getMessage();
      if isempty(contact_data)
        lfoot_contact_state = 0;
        rfoot_contact_state = 0;
      else
        msg = drc.foot_contact_estimate_t(contact_data);
        lfoot_contact_state = msg.left_contact > 0.5;
        rfoot_contact_state = msg.right_contact > 0.5;
      end
      
%       if lfoot_contact_state
%         Kp(obj.l_ank,obj.l_ank) = 0*eye(2);
%         Kd(obj.l_ank,obj.l_ank) = 0*eye(2);
%       end
%       if rfoot_contact_state
%         Kp(obj.r_ank,obj.r_ank) = 0*eye(2);
%         Kd(obj.r_ank,obj.r_ank) = 0*eye(2);
%       end
%       
      approx_args = {};
      for j = 1:length(cdata.link_constraints)
        if ~isempty(cdata.link_constraints(j).traj)
          pos = fasteval(cdata.link_constraints(j).traj,t);
%           pos(3) = pos(3) - cdata.trans_drift(3);
          pos(1:3) = pos(1:3) - cdata.trans_drift;
          approx_args(end+1:end+3) = {cdata.link_constraints(j).link_ndx, cdata.link_constraints(j).pt, pos};
        end
      end
      
      % note: we should really only try to control COM position when in
      % contact with the environment
      com = fasteval(cdata.comtraj,t);
      compos = [com(1:2) - cdata.trans_drift(1:2);nan];

      q_des = linearIK(obj.robot,q,0,compos,approx_args{:},obj.ikoptions);
      
      err_q = q_des - q;
      nrmerr = norm(err_q,1);
      if nrmerr > obj.max_nrm_err
        err_q = obj.max_nrm_err * err_q / nrmerr;
			end
			
			y = max(-100*ones(obj.nq,1),min(100*ones(obj.nq,1),obj.Kp.*err_q - obj.Kd.*qd));
      if infocount > 0
  		  save(sprintf('pd_dump_t=%2.3f.mat',t),'q','q_des','err_q','y','compos','approx_args');
      end
      
      
		end
  end
  
end
