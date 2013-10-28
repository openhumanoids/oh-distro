classdef ApproximateIKBlock < MIMODrakeSystem
  % outputs a q_desired (including floating dofs)
  properties
    nq;
    controller_data; % pointer to shared data handle containing qtraj
    ikoptions;
    robot;
    ik_qnom;
  end
  
  methods
    function obj = ApproximateIKBlock(r,controller_data,options)
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
      
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.004;
      end
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate
     
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
      obj.ikoptions = IKoptions(r);
      obj.ikoptions = obj.ikoptions.setQ(diag(cost(1:obj.nq)));
      obj.ik_qnom = q_nom;
      % Prevent the knee from locking
      %[obj.ikoptions.jointLimitMin, obj.ikoptions.jointLimitMax] = r.getJointLimits();
      %joint_names = r.getStateFrame.coordinates(1:r.getNumDOF());
      %obj.ikoptions.jointLimitMin(~cellfun(@isempty,strfind(joint_names,'kny'))) = 0.6;


      obj.robot = r;
    end
   
    function q_des=mimoOutput(obj,t,~,varargin)
      x = varargin{2};
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);

      obj.ik_qnom = varargin{1};
      cdata = obj.controller_data.data;
      
      approx_args = {};
%       approx_args_bk = {};
      for j = 1:length(cdata.link_constraints)
        if ~isempty(cdata.link_constraints(j).traj)
          pos = fasteval(cdata.link_constraints(j).traj,t);
%           pos(3) = pos(3) - cdata.trans_drift(3);
          pos(1:3) = pos(1:3) - cdata.trans_drift;
%           approx_args_bk(end+1:end+3) = {cdata.link_constraints(j).link_ndx, cdata.link_constraints(j).pt, pos};
          approx_args = [approx_args,wrapDeprecatedConstraint(obj.robot,cdata.link_constraints(j).link_ndx,cdata.link_constraints(j).pt,pos,struct('use_mex',true))];
        else
          pos_min = fasteval(cdata.link_constraints(j).min_traj,t);
%           pos_min(3) = pos_min(3) - cdata.trans_drift(3);
          pos_min(1:3) = pos_min(1:3) - cdata.trans_drift;
          pos_max = fasteval(cdata.link_constraints(j).max_traj,t);
%           pos_max(3) = pos_max(3) - cdata.trans_drift(3);
          pos_max(1:3) = pos_max(1:3) - cdata.trans_drift;
%           approx_args_bk(end+1:end+3) = {cdata.link_constraints(j).link_ndx, cdata.link_constraints(j).pt, struct('min', pos_min, 'max', pos_max)};
          approx_args = [approx_args,wrapDeprecatedConstraint(obj.robot,cdata.link_constraints(j).link_ndx,cdata.link_constraints(j).pt,struct('min', pos_min, 'max', pos_max),struct('use_mex',true))];
        end
      end
      
      % note: we should really only try to control COM position when in
      % contact with the environment
      com = fasteval(cdata.comtraj,t);
      if length(com)==3
        compos = [com(1:2) - cdata.trans_drift(1:2);com(3)];
      else
        compos = [com(1:2) - cdata.trans_drift(1:2);nan];
      end
      kc_com = wrapDeprecatedConstraint(obj.robot,0,[],compos,struct('use_mex',true));
      approx_args = [approx_args,kc_com];
      [q_des,info] = approximateIKmex(obj.robot.getMexModelPtr,q,obj.ik_qnom,approx_args{:},obj.ikoptions);
%       obj.ikoptions_bk.use_mex = false;
%       [q_des,info] = approximateIK_bk(obj.robot,q,0,compos,approx_args_bk{:},obj.ikoptions_bk);
%       max(abs((q_des-q_des_bk)))
      if info
        fprintf(1,'warning: approximate IK failed.  calling IK\n');
        q_des = inverseKin(obj.robot,q,obj.ik_qnom,approx_args{:},obj.ikoptions);
      end
    end
  end
  
end
