classdef QPController < MIMODrakeSystem

  methods
  function obj = QPController(r,controller_data,options)
    % @param r atlas instance
    % @param options structure for specifying objective weight (w), slack
    % variable limits (slack_limit), and input cost (R)
    typecheck(r,'Atlas');
    typecheck(controller_data,'SharedDataHandle');

    QPController.check_ctrl_data(controller_data)
    
    if nargin>2
      typecheck(options,'struct');
    else
      options = struct();
    end
    
    qddframe = AtlasCoordinates(r); % input frame for desired qddot 
    hand_ft_frame = AtlasHandForceTorque();

    if isfield(options,'multi_robot')
      warning('Bullet contact not currently supported for efficiency reasons. Ignoring multi robot.');
%       typecheck(options.multi_robot,'TimeSteppingRigidBodyManipulator');
%       fr = options.multi_robot.getStateFrame;
%       % IMPORTANT NOTE: I'm assuming the atlas state is always the first
%       % frame in a multi coordinate frame
%       if typecheck(fr,'MultiCoordinateFrame')
%         input_frame = MultiCoordinateFrame({qddframe,hand_ft_frame,options.multi_robot.getStateFrame.frame{:}});
%         num_state_fr = length(options.multi_robot.getStateFrame.frame);
%       else
%         input_frame = MultiCoordinateFrame({qddframe,hand_ft_frame,options.multi_robot.getStateFrame});
%         num_state_fr = 1;
%       end
      input_frame = MultiCoordinateFrame({qddframe,hand_ft_frame,r.getStateFrame});
      num_state_fr = 1;
    else
      input_frame = MultiCoordinateFrame({qddframe,hand_ft_frame,r.getStateFrame});
      num_state_fr = 1;
    end
    
    output_frame = r.getInputFrame();
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setSampleTime(obj,[.005;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.controller_data = controller_data;
    obj.num_state_frames = num_state_fr;
    
%     if isfield(options,'multi_robot')
%       obj.multi_robot = options.multi_robot;
%     else
%       obj.multi_robot = 0;
%     end
    obj.multi_robot = 0;
   
    % weight for desired qddot objective term
    if isfield(options,'w')
      typecheck(options.w,'double');
      sizecheck(options.w,1);
      obj.w = options.w;
    else
      obj.w = 0.1;
    end
    
    % hard bound on slack variable values
    if isfield(options,'slack_limit')
      typecheck(options.slack_limit,'double');
      sizecheck(options.slack_limit,1);
      obj.slack_limit = options.slack_limit;
    else
      obj.slack_limit = 10;
    end
    
    nu = getNumInputs(r);
    % input cost term: u'Ru
    if ~isfield(options,'R')
      obj.R = 1e-6*eye(nu);
    else
      typecheck(options.R,'double');
      sizecheck(options.R,[nu,nu]);
      obj.R = options.R;
    end
    
    if ~isfield(options,'lcm_foot_contacts')
      obj.lcm_foot_contacts=true; % listen for foot contacts over LCM
    else
      typecheck(options.lcm_foot_contacts,'logical');
      obj.lcm_foot_contacts=options.lcm_foot_contacts;
    end
    
    if isfield(options,'debug')
      typecheck(options.debug,'logical');
      sizecheck(options.debug,1);
      obj.debug = options.debug;
    else
      obj.debug = false;
    end

    if isfield(options,'use_mex')
      % 0 - no mex
      % 1 - use mex
      % 2 - run mex and non-mex and valuecheck the result
      sizecheck(options.use_mex,1);
      obj.use_mex = uint32(options.use_mex);
      rangecheck(obj.use_mex,0,2);
      if (obj.use_mex && exist('QPControllermex')~=3)
        error('can''t find QPControllermex.  did you build it?');
      end
    else
      obj.use_mex = 1;
    end

    if isfield(options,'use_hand_ft')
      obj.use_hand_ft = options.use_hand_ft;
    else
      obj.use_hand_ft = false;
    end
    
    % specifies whether or not to solve QP for all DOFs or just the
    % important subset
    if (isfield(options,'full_body_opt'))
      typecheck(options.full_body_opt,'logical');
    else
      options.full_body_opt = true;
    end
    
    if ~options.full_body_opt
      % perform unconstrained minimization to compute accelerations for a 
      % subset of atlas DOF, then solve for inputs (then threshold).
      % generally these should be the joints for which the columns of the 
      % contact jacobian are zero. The remaining dofs are indexed in con_dof.
      state_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      obj.free_dof = find(~cellfun(@isempty,strfind(state_names,'arm')) + ...
                    ~cellfun(@isempty,strfind(state_names,'neck')));
      obj.con_dof = setdiff(1:getNumDOF(r),obj.free_dof)';
      
      input_names = r.getInputFrame.coordinates;
      obj.free_inputs = find(~cellfun(@isempty,strfind(input_names,'arm')) | ~cellfun(@isempty,strfind(input_names,'neck')));
      obj.con_inputs = setdiff(1:getNumInputs(r),obj.free_inputs)';
    else
      obj.free_dof = [];
      obj.con_dof = 1:getNumDOF(r);
      obj.free_inputs = [];
      obj.con_inputs = 1:getNumInputs(r);
    end
    
    obj.lc = lcm.lcm.LCM.getSingleton();
    obj.rfoot_idx = findLinkInd(r,'r_foot');
    obj.lfoot_idx = findLinkInd(r,'l_foot');
    obj.rhand_idx = findLinkInd(r,'r_hand');
    obj.lhand_idx = findLinkInd(r,'l_hand');

    if obj.lcm_foot_contacts
      obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
      obj.lc.subscribe('FOOT_CONTACT_ESTIMATE',obj.contact_est_monitor);
    end % else estimate contact via kinematics
    
    if obj.solver==1 % use cplex
      obj.solver_options = cplexoptimset('cplex');
      obj.solver_options.diagnostics = 'on';
      obj.solver_options.maxtime = 0.01;
      % QP method: 
      %   0 	Automatic (default)
      %   1 	Primal Simplex
      %   2 	Dual Simplex
      %   3 	Network Simplex
      %   4 	Barrier
      %   5 	Sifting
      %   6 	Concurrent
      obj.solver_options.qpmethod = 4; 
      
    else % use gurobi

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% NOTE: these parameters need to be set in QPControllermex.cpp, too %%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      obj.solver_options.outputflag = 0; % not verbose
      obj.solver_options.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
      obj.solver_options.presolve = 0;
%       obj.solver_options.prepasses = 1;

      if obj.solver_options.method == 2
        obj.solver_options.bariterlimit = 20; % iteration limit
        obj.solver_options.barhomogeneous = 0; % 0 off, 1 on
        obj.solver_options.barconvtol = 5e-4;
      end
    end  
    
    if (obj.use_mex>0)
      terrain = getTerrain(r);
      if isa(terrain,'DRCTerrainMap') 
        terrain_map_ptr = terrain.map_handle.getPointerForMex();
      else
        terrain_map_ptr = 0;
      end
      if obj.multi_robot==0
        multi_robot_ptr = 0;
      else
        multi_robot_ptr = obj.robot.getMexModelPtr.getData();
      end
      obj.mex_ptr = SharedDataHandle(QPControllermex(0,obj,obj.robot.getMexModelPtr.getData(),getB(obj.robot),r.umin,r.umax,terrain_map_ptr,multi_robot_ptr,obj.rfoot_idx,obj.lfoot_idx));
%       obj.mex_ptr = SharedDataHandle(QPControllermex(0,obj,obj.robot.getMexModelPtr.getData(),getB(obj.robot),r.umin,r.umax));
    end

    obj.num_body_contacts=zeros(getNumBodies(r),1);
    for i=1:getNumBodies(r)
      obj.num_body_contacts(i) = length(getBodyContacts(r,i));
    end
    
    
    if isa(getTerrain(r),'DRCFlatTerrainMap')
      obj.using_flat_terrain = true;      
    else
      obj.using_flat_terrain = false;
    end
  end

  end
  
  methods (Static)
    function check_ctrl_data(ctrl_data)
      if ~isfield(ctrl_data.data,'D')
        % assumed  ZMP system
        hddot = 0; % could use estimated comddot here
        ctrl_data.setField('D',-0.89/(hddot+9.81)*eye(2)); % TMP hard coding height here. Could be replaced with htraj from planner
        % or current height above height map;
      end

      ctrl_data = ctrl_data.data;
      
      % i've made the following assumptions to make things fast.  we can soften
      % them later as desired.  - Russ
      assert(isnumeric(ctrl_data.Qy));
%       sizecheck(ctrl_data.Qy,[2 2]); % commented out by sk--some of the
%       quasistatic systems pass 4x4 Qs
      assert(isnumeric(ctrl_data.R));
      sizecheck(ctrl_data.R,[2 2]);
      assert(isnumeric(ctrl_data.C));
      
      assert(isnumeric(ctrl_data.S));
      sizecheck(ctrl_data.S,[4 4]);
      assert(isnumeric(ctrl_data.x0));
      sizecheck(ctrl_data.x0,[4 1]);
      assert(isnumeric(ctrl_data.u0));
      if ctrl_data.is_time_varying
        assert(isa(ctrl_data.s1,'Trajectory'));
        assert(isa(ctrl_data.y0,'Trajectory'));
      else
        assert(isnumeric(ctrl_data.s1));
        assert(isnumeric(ctrl_data.y0));
%        sizecheck(ctrl_data.supports,1);  % this gets initialized to zero
%        in constructors.. but doesn't get used.  would be better to
%        enforce it.
      end       
      sizecheck(ctrl_data.s1,[4 1]);
      assert(isnumeric(ctrl_data.s2));
      sizecheck(ctrl_data.s2,1);
      assert(isnumeric(ctrl_data.mu));
      assert(islogical(ctrl_data.ignore_terrain));
    end
  end
  
  methods
    
  function y=mimoOutput(obj,t,~,varargin)
%    out_tic = tic;
    ctrl_data = obj.controller_data.data;
    
%    QPController.check_ctrl_data(ctrl_data);  % todo: remove this after all of the DRC Controllers call it reliably on their initialize method
  
    q_ddot_des = varargin{1};
    hand_ft = varargin{2};
    % IMPORTANT NOTE: I'm assuming the atlas state is always the first
    % frame in a multi coordinate frame 
    x = varargin{3};
    
    r = obj.robot;
    nq = getNumDOF(r); 
    q = x(1:nq); 
    qd = x(nq+(1:nq)); 

%     q_multi = [];
%     for i=1:obj.num_state_frames-1
%       xi = varargin{3+i};
%       q_multi = [q_multi; xi(1:end/2)];
%     end
    
%     Num = [-0.0161998291483754,-0.0211451622014118,0.0207009617283916,0.125779402410861,0.247057672241705,0.301449966246763,0.247057672241705,0.125779402410861,0.0207009617283916,-0.0211451622014118,-0.0161998291483754];
%     Num = [-0.000479358954034654,7.08339282197222e-05,0.00476272575802094,0.0184304925452958,0.0447366690382284,0.0825921917406667,0.124222528702383,0.157134664685447,0.169687809650027,0.157134664685447,0.124222528702383,0.0825921917406667,0.0447366690382284,0.0184304925452958,0.00476272575802094,7.08339282197222e-05,-0.000479358954034654];
%     Num = [-0.00708947721498600,-0.0109352396875095,-0.0122908847163248,-0.00497044418042090,0.0151465638562752,0.0486147429458304,0.0909443688764279,0.133146505097372,0.164402269282922,0.175940834823367,0.164402269282922,0.133146505097372,0.0909443688764279,0.0486147429458304,0.0151465638562752,-0.00497044418042090,-0.0122908847163248,-0.0109352396875095,-0.00708947721498645];
%     Num = [-0.00106596876350730,0.000783329836666458,0.00290044773164991,0.00683518271911500,0.0129147048037398,0.0212675134522692,0.0317156030692218,0.0437274552796116,0.0564299663108208,0.0687073812494054,0.0793356376662851,0.0871721639053985,0.0913335369454345,0.0913335369454345,0.0871721639053985,0.0793356376662851,0.0687073812494054,0.0564299663108208,0.0437274552796116,0.0317156030692218,0.0212675134522692,0.0129147048037398,0.00683518271911500,0.00290044773164991,0.000783329836666458,-0.00106596876350730];
%     Num = [0.00299756178194235,0.00293543517245433,0.00426411360001696,0.00588940605349714,0.00782107025494986,0.0100568052901440,0.0125823027484534,0.0153709358339710,0.0183830321637842,0.0215687053490253,0.0248626313438620,0.0281954486790890,0.0314834849720075,0.0346493764787068,0.0376052528255108,0.0402698968175043,0.0425680172486468,0.0444304351178032,0.0458028723509376,0.0466440635635022,0.0469283423025935,0.0466440635635022,0.0458028723509376,0.0444304351178032,0.0425680172486468,0.0402698968175043,0.0376052528255108,0.0346493764787068,0.0314834849720075,0.0281954486790890,0.0248626313438620,0.0215687053490253,0.0183830321637842,0.0153709358339710,0.0125823027484534,0.0100568052901440,0.00782107025494986,0.00588940605349714,0.00426411360001696,0.00293543517245433,0.00299756178194235];
%     if isempty(fidx)
%       fidx = 1;
%     else
%       fidx = mod(fidx,length(Num))+1;
%     end
% 
%     if isempty(qd_hist)
%       qd_hist = zeros(nq,length(Num));
%     end
%     
%     qd_hist(:,fidx) = qd;
%     res = filter(Num,1,qd_hist(:,[fidx+1:length(Num),1:fidx])');
%     qd_filt = res(end,:)';
        
    if obj.lcm_foot_contacts
      % get foot contact state over LCM
      contact_data = obj.contact_est_monitor.getMessage();
      if isempty(contact_data)
        lfoot_contact_state = 0;
        rfoot_contact_state = 0;
      else
        msg = drc.foot_contact_estimate_t(contact_data);
        lfoot_contact_state = msg.left_contact;
        rfoot_contact_state = msg.right_contact;
      end
    else
      lfoot_contact_state=0;
      rfoot_contact_state=0;
    end
    
    %----------------------------------------------------------------------
    % Linear system stuff for zmp/com control -----------------------------
    A_ls = ctrl_data.A; % always TI
    B_ls = ctrl_data.B; % always TI
    Qy = ctrl_data.Qy;
    R_ls = ctrl_data.R;
    C_ls = ctrl_data.C;
    D_ls = ctrl_data.D;
    S = ctrl_data.S;
    s2 = ctrl_data.s2;
    x0 = ctrl_data.x0 - [ctrl_data.trans_drift(1:2);0;0]; % TESTING, ADDED BY SCOTT
    u0 = ctrl_data.u0;
    if (ctrl_data.is_time_varying)
      s1 = eval(ctrl_data.s1,t);
      y0 = eval(ctrl_data.y0,t) - ctrl_data.trans_drift(1:2); % TESTING, ADDED BY SCOTT
      
      %----------------------------------------------------------------------
      % extract current supports
      supp_idx = find(ctrl_data.support_times<=t,1,'last');
      supp = ctrl_data.supports(supp_idx);
    else
      s1 = ctrl_data.s1;
      y0 = ctrl_data.y0 - ctrl_data.trans_drift(1:2); % TESTING, ADDED BY SCOTT
      
      supp = ctrl_data.supports;
    end
    mu = ctrl_data.mu;
    R_DQyD_ls = R_ls + D_ls'*Qy*D_ls;

    desired_supports = supp.bodies;

    
    %----------------------------------------------------------------------
    % check for cases that I haven't implemented yet in mex
    if (obj.use_mex~=0)  
      if any(supp.contact_surfaces~=0)
        error('multi-robot contact not supported by mex version yet');
      end
    end
    
    contact_threshold = 0.001; % m
    if (obj.use_mex==0 || obj.use_mex==2)

      % Change in logic here due to recent tests with heightmap noise
      % for now, we will do a logical OR of the force-based sensor and the
      % kinematic criterion for foot contacts
      %
      % another option would be to limit forces on the feet when kinematics
      % says 'contact', but force sensors do not. when both agree, allow full
      % forces on the feet
      kinsol = doKinematics(r,q,false,true);

%       % REMOVING SUPPORT FOR MULTI ROBOTS FOR NOW
%       if any(supp.contact_surfaces~=0) && isa(obj.multi_robot,'TimeSteppingRigidBodyManipulator')
%         kinsol_multi = doKinematics(obj.multi_robot,[q;q_multi],false,true); % for now assume the same state frame
%       end
      
      num_desired_contacts = supp.num_contact_pts;
    
      % get active contacts
      phi = contactConstraints(r,kinsol,desired_supports,supp.contact_pts);

%       phi = zeros(sum(num_desired_contacts),1);
%       c_pre = 0;
%       for j=1:length(supp.bodies)
%         if supp.contact_surfaces(j) == 0
%           phi(c_pre+(1:num_desired_contacts(j))) = contactConstraints(r,kinsol,desired_supports(j),supp.contact_pts{j});
%         elseif isa(obj.multi_robot,'TimeSteppingRigidBodyManipulator')
%         % use bullet collision between bodies
%       % REMOVING SUPPORT FOR MULTI ROBOTS FOR NOW
%          phi(c_pre+(1:num_desired_contacts(j))) = pairwiseContactConstraints(obj.multi_robot,kinsol_multi,desired_supports(j),supp.contact_surfaces(j),supp.contact_pts{j});
%         else
%           error('QPController: multi_robot not defined, cannot call pairwise contact constraints');
%         end
%         c_pre = c_pre + num_desired_contacts(j);
%       end

      lfoot_contact_state_kin = 0;
      rfoot_contact_state_kin = 0;
      if ~ctrl_data.ignore_terrain
        % check foot contacts via kinematics
        if any(obj.lfoot_idx==desired_supports)
          lfoot_desired_idx = find(obj.lfoot_idx==desired_supports);
          c_pre = sum(num_desired_contacts(1:lfoot_desired_idx-1));
          if any(phi(c_pre+(1:num_desired_contacts(lfoot_desired_idx)))<=contact_threshold)
            lfoot_contact_state_kin = 1;
          end
        end

        if any(obj.rfoot_idx==desired_supports)
          rfoot_desired_idx = find(obj.rfoot_idx==desired_supports);
          c_pre = sum(num_desired_contacts(1:rfoot_desired_idx-1));
          if any(phi(c_pre+(1:num_desired_contacts(rfoot_desired_idx)))<=contact_threshold)
            rfoot_contact_state_kin = 1;
          end
        end
      end
      
      lfoot_contact_state = lfoot_contact_state || lfoot_contact_state_kin;
      rfoot_contact_state = rfoot_contact_state || rfoot_contact_state_kin;
      
      active_supports = [];
      active_surfaces = [];
      active_contact_pts = {};
      num_active_contacts = [];
      if any(desired_supports==obj.lfoot_idx) && lfoot_contact_state > 0.5
        active_supports = [active_supports; obj.lfoot_idx];
        active_surfaces = [active_surfaces; supp.contact_surfaces(lfoot_desired_idx)];
        active_contact_pts{length(active_supports)} = supp.contact_pts{lfoot_desired_idx};
        num_active_contacts = [num_active_contacts; length(supp.contact_pts{lfoot_desired_idx})];
      end
      if any(desired_supports==obj.rfoot_idx) && rfoot_contact_state > 0.5
        active_supports = [active_supports; obj.rfoot_idx];
        active_surfaces = [active_surfaces; supp.contact_surfaces(rfoot_desired_idx)];
        active_contact_pts{length(active_supports)} = supp.contact_pts{rfoot_desired_idx};
        num_active_contacts = [num_active_contacts; length(supp.contact_pts{rfoot_desired_idx})];
      end
      
      %----------------------------------------------------------------------
      % END CODE THAT TREATS FEET DIFFERENTLY -------------------------------
      
      c_pre = 0;
      for i=1:length(desired_supports)
        if desired_supports(i)~=obj.lfoot_idx && desired_supports(i)~=obj.rfoot_idx
          if any(phi(c_pre+(1:num_desired_contacts(i)))<=contact_threshold)
            active_supports = [active_supports; desired_supports(i)];
            active_surfaces = [active_surfaces; supp.contact_surfaces(i)];
            active_contact_pts{length(active_supports)} = supp.contact_pts{i};
            num_active_contacts = [num_active_contacts; length(supp.contact_pts{i})];
          end
        end
        c_pre = c_pre + num_desired_contacts(i);
      end
      
      %----------------------------------------------------------------------
      % Disable hand force/torque contribution to dynamics as necessary
      if (~obj.use_hand_ft)
        hand_ft=0*hand_ft;
      else
        if any(active_supports==obj.lhand_idx)
          hand_ft(1:6)=0;
        end
        if any(active_supports==obj.rhand_idx)
          hand_ft(7:12)=0;
        end
      end
          
      %----------------------------------------------------------------------
      
      nu = getNumInputs(r);
      nq = getNumDOF(r);
      dim = 3; % 3D
      nd = 4; % for friction cone approx, hard coded for now
      nq_free = length(obj.free_dof);
      nq_con = length(obj.con_dof);
      nu_con = length(obj.con_inputs);

      kinsol = doKinematics(r,q,false,true,qd);
      
      [H,C,B] = manipulatorDynamics(r,q,qd);
      
      [~,Jlhand] = forwardKin(r,kinsol,obj.lhand_idx,zeros(3,1),1);
      [~,Jrhand] = forwardKin(r,kinsol,obj.rhand_idx,zeros(3,1),1);
      C = C + Jlhand'*hand_ft(1:6) + Jrhand'*hand_ft(7:12);
      
      H_con = H(obj.con_dof,:);
      C_con = C(obj.con_dof);
      B_con = B(obj.con_dof,obj.con_inputs);
      
      if nq_free > 0
        H_free = H(obj.free_dof,:);
        C_free = C(obj.free_dof);
        B_free = B(obj.free_dof,obj.free_inputs);
      end
      
      [xcom,J] = getCOM(r,kinsol);
      Jdot = forwardJacDot(r,kinsol,0);
      J = J(1:2,:); % only need COM x-y
      Jdot = Jdot(1:2,:);
      
      if ~isempty(active_supports)
        nc = sum(num_active_contacts);
        [~,Jz,D_] = contactConstraints(r,kinsol,active_supports,active_contact_pts);
%         phi = zeros(nc,1);
%         Jz = zeros(nc,nq);
%         c_pre = 0;
%         for j=1:length(active_supports)
%           if active_surfaces(j) == 0
%             [phi(c_pre+(1:num_desired_contacts(j))),Jz(c_pre+(1:num_desired_contacts(j)),:),D__] = contactConstraints(r,kinsol,active_supports(j),active_contact_pts{j});
%           % REMOVING SUPPORT FOR MULTI ROBOTS FOR NOW
%           elseif isa(obj.multi_robot,'TimeSteppingRigidBodyManipulator')
%             % use bullet collision between bodies
%             [phi(c_pre+(1:num_desired_contacts(j))),Jz(c_pre+(1:num_desired_contacts(j)),:),D__] = pairwiseContactConstraints(obj.multi_robot,kinsol_multi,active_supports(j),active_surfaces(j),active_contact_pts{j});
%           else
%             error('QPController: multi_robot not defined, cannot call pairwise contact constraints');
%           end
%           c_pre = c_pre + num_desired_contacts(j);
%         
%           % kinda gross
%           if j==1
%             D_=D__;
%           else
%             for k=1:nd
%               D_{k} = [D_{k}; D__{k}];
%             end
%           end
%         
%         end
      else
        nc = 0;
      end
      neps = nc*dim;
      %     neps = length(active_supports)*2*dim;
      
      if nc > 0
        [cpos,Jp,Jpdot] = contactPositionsJdot(r,kinsol,active_supports,active_contact_pts);
        %       Jp=zeros(neps,nq);
        %       Jpdot=zeros(neps,nq);
        %       for k=1:length(active_supports)
        %         [~,Jp((k-1)*2*dim+(1:2*dim),:)] = forwardKin(r,kinsol,active_supports(k),[[1;0;0],[0;1;0]],0);
        %         Jpdot((k-1)*2*dim+(1:2*dim),:) = forwardJacDot(r,kinsol,active_supports(k),[[1;0;0],[0;1;0]]);
        %       end
        Jp = sparse(Jp(:,obj.con_dof));
        Jpdot = sparse(Jpdot(:,obj.con_dof));
        
        Jz = sparse(Jz(:,obj.con_dof)); % only care about active contacts
        
        % D_ is the parameterization of the polyhedral approximation of the
        %    friction cone, in joint coordinates (figure 1 from Stewart96)
        %    D{k}(i,:) is the kth direction vector for the ith contact (of nC)
        % Create Dbar such that Dbar(:,(k-1)*nd+i) is ith direction vector for
        % the kth contact point
        %
        % OPT---this isn't necessary
        D = cell(1,nc);
        for k=1:nc
          for i=1:nd
            D{k}(:,i) = D_{i}(k,obj.con_dof)';
          end
        end
        Dbar = sparse([D{:}]);
      end      
        
      if (nc>0)
        xlimp = [xcom(1:2); J*qd]; % state of LIP model
        x_bar = xlimp - x0;      
      end
      
      
      %----------------------------------------------------------------------
      % Free DOF cost function ----------------------------------------------

      if nq_free > 0
        if nc > 0
          % approximate quadratic cost for free dofs with the appropriate matrix block
          Hqp = J(:,obj.free_dof)'*R_DQyD_ls*J(:,obj.free_dof);
          Hqp = Hqp + obj.w*eye(nq_free);

          fqp = xlimp'*C_ls'*Qy*D_ls*J(:,obj.free_dof);
          fqp = fqp + qd(obj.free_dof)'*Jdot(:,obj.free_dof)'*R_DQyD_ls*J(:,obj.free_dof);
          fqp = fqp + (x_bar'*S + 0.5*s1')*B_ls*J(:,obj.free_dof);
          fqp = fqp - u0'*R_ls*J(:,obj.free_dof);
          fqp = fqp - y0'*Qy*D_ls*J(:,obj.free_dof);
          fqp = fqp - obj.w*q_ddot_des(obj.free_dof)';
        else
          Hqp = eye(nq_free);
          fqp = -q_ddot_des(obj.free_dof)';
        end

        % solve for qdd_free unconstrained
        qdd_free = -Hqp\fqp';
      end      
        
    
      %----------------------------------------------------------------------
      % Build handy index matrices ------------------------------------------
      
      nf = nc+nc*nd; % number of contact force variables
      nparams = nq_con+nu_con+nf+neps;
      Iqdd = zeros(nq_con,nparams); Iqdd(:,1:nq_con) = eye(nq_con);
      Iu = zeros(nu_con,nparams); Iu(:,nq_con+(1:nu_con)) = eye(nu_con);
      Iz = zeros(nc,nparams); Iz(:,nq_con+nu_con+(1:nc)) = eye(nc);
      Ibeta = zeros(nc*nd,nparams); Ibeta(:,nq_con+nu_con+nc+(1:nc*nd)) = eye(nc*nd);
      Ieps = zeros(neps,nparams);
      Ieps(:,nq_con+nu_con+nc+nc*nd+(1:neps)) = eye(neps);
      
      
      %----------------------------------------------------------------------
      % Set up problem constraints ------------------------------------------
      
      lb = [-1e3*ones(1,nq_con) r.umin(obj.con_inputs)' zeros(1,nf)   -obj.slack_limit*ones(1,neps)]'; % qddot/input/contact forces/slack vars
      ub = [ 1e3*ones(1,nq_con) r.umax(obj.con_inputs)' 500*ones(1,nf) obj.slack_limit*ones(1,neps)]';
      
      Aeq_ = cell(1,2);
      beq_ = cell(1,2);
      Ain_ = cell(1,nc);
      bin_ = cell(1,nc);
      
      % constrained dynamics
      if nc>0
        Aeq_{1} = H_con(:,obj.con_dof)*Iqdd - B_con*Iu - Jz'*Iz - Dbar*Ibeta;
      else
        Aeq_{1} = H_con(:,obj.con_dof)*Iqdd - B_con*Iu;
      end
      if nq_free > 0
        beq_{1} = -C_con - H_con(:,obj.free_dof)*qdd_free;
      else
        beq_{1} = -C_con;
      end
      
      if nc > 0
        % relative acceleration constraint
        Aeq_{2} = Jp*Iqdd + Ieps;
        beq_{2} = -Jpdot*qd(obj.con_dof) - 1.0*Jp*qd(obj.con_dof);
        
        % linear friction constraints
        for i=1:nc
          Ain_{i} = -mu*Iz(i,:) + sum(Ibeta((i-1)*nd+(1:nd),:));
          bin_{i} = 0;
        end
      end
      
      % linear equality constraints: Aeq*alpha = beq
      Aeq = sparse(vertcat(Aeq_{:}));
      beq = vertcat(beq_{:});
      
      % linear inequality constraints: Ain*alpha <= bin
      Ain = sparse(vertcat(Ain_{:}));
      bin = vertcat(bin_{:});

    
      %----------------------------------------------------------------------
      % QP cost function ----------------------------------------------------
      %
      %  min: quad(Jdot*qd + J*qdd,R_ls)+quad(C*x_bar+D*(Jdot*qd + J*qdd),Qy) + (2*x_bar'*S + s1')*(A*x_bar + B*(Jdot*qd + J*qdd-u0)) + w*quad(qddot_ref - qdd) + quad(u,R) + quad(epsilon)
      
      if nc > 0
        Hqp = Iqdd'*J(:,obj.con_dof)'*R_DQyD_ls*J(:,obj.con_dof)*Iqdd;
        Hqp(1:nq_con,1:nq_con) = Hqp(1:nq_con,1:nq_con) + obj.w*eye(nq_con);

        fqp = xlimp'*C_ls'*Qy*D_ls*J(:,obj.con_dof)*Iqdd;
        fqp = fqp + qd(obj.con_dof)'*Jdot(:,obj.con_dof)'*R_DQyD_ls*J(:,obj.con_dof)*Iqdd;
        fqp = fqp + (x_bar'*S + 0.5*s1')*B_ls*J(:,obj.con_dof)*Iqdd;
        fqp = fqp - u0'*R_ls*J(:,obj.con_dof)*Iqdd;
        fqp = fqp - y0'*Qy*D_ls*J(:,obj.con_dof)*Iqdd;
        fqp = fqp - obj.w*q_ddot_des(obj.con_dof)'*Iqdd;

        % quadratic slack var cost 
        Hqp(nparams-neps+1:end,nparams-neps+1:end) = 0.001*eye(neps); 
      else
        Hqp = Iqdd'*Iqdd;
        fqp = -q_ddot_des(obj.con_dof)'*Iqdd;
      end
      
      % quadratic input cost
      Hqp(nq_con+(1:nu_con),nq_con+(1:nu_con)) = obj.R(obj.con_inputs,obj.con_inputs);
      

      %----------------------------------------------------------------------
      % Solve QP ------------------------------------------------------------
      
      if obj.solver==1
        % CURRENTLY CRASHES MATLAB ON MY MACHINE -sk
        alpha = cplexqp(Hqp,fqp,Ain,bin,Aeq,beq,lb,ub,[],obj.solver_options);
        
      else
        model.Q = sparse(Hqp);
        model.obj = 2*fqp;
        model.A = [Aeq; Ain];
        model.rhs = [beq; bin];
        model.sense = [obj.eq_array(1:length(beq)); obj.ineq_array(1:length(bin))];
        model.lb = lb;
        model.ub = ub;
        
        if (any(any(isnan(model.Q))) || any(isnan(model.obj)) || any(any(isnan(model.A))) || any(isnan(model.rhs)) || any(isnan(model.lb)) || any(isnan(model.ub)))
          keyboard;
        end

%       Q=full(Hqp);
%       c=2*fqp;
%       Aeq = full(Aeq);
%       Ain = full(Ain);
%       save(sprintf('data/model_t_%2.3f.mat',t),'Q','c','Aeq','beq','Ain','bin','lb','ub');
%       qp_tic = tic;
        result = gurobi(model,obj.solver_options);
%       qp_toc = toc(qp_tic);
%       fprintf('QP solve: %2.4f\n',qp_toc);
        alpha = result.x;
        
      end

      %----------------------------------------------------------------------
      % Solve for free inputs -----------------------------------------------
      if nq_free > 0
        qdd = zeros(nq,1);
        qdd(obj.free_dof) = qdd_free;
        qdd(obj.con_dof) = alpha(1:nq_con);
        
        u_free = B_free\(H_free*qdd + C_free);
        u = zeros(nu,1);
        u(obj.free_inputs) = u_free;
        u(obj.con_inputs) = alpha(nq_con+(1:nu_con));
        
        % saturate inputs
        y = max(r.umin,min(r.umax,u));
      else
        y = alpha(nq+(1:nu));
      end
      
      if (obj.use_mex==2)
        des.y = y;
      end
      
      % compute V,Vdot for controller status updates
      if (nc>0)
        V = x_bar'*S*x_bar + s1'*x_bar + s2;
        if nq_free > 0
          qdd = zeros(nq,1);
          qdd(obj.free_dof) = qdd_free;
          qdd(obj.con_dof) = alpha(1:nq_con);
        else
          qdd = alpha(1:nq);
        end
        Vdot = (2*x_bar'*S + s1')*(A_ls*x_bar + B_ls*(Jdot*qd + J*qdd));
      end
      
    end
  
    if (obj.use_mex==1)
      if ctrl_data.ignore_terrain
        contact_threshold =-1;       
      end
      if obj.using_flat_terrain
        height = getTerrainHeight(r,[0;0]); % get height from DRCFlatTerrainMap
      else
        height = 0;
      end
      [y,Vdot,active_supports] = QPControllermex(obj.mex_ptr.getData(),q_ddot_des,x,desired_supports,A_ls,B_ls,Qy,R_ls,C_ls,D_ls,S,s1,x0,u0,y0,mu,rfoot_contact_state,lfoot_contact_state,contact_threshold,height);
      V = 0; % don't compute V for mex yet (will we ever use this?)
    end

    if ~isempty(active_supports)
      setField(obj.controller_data,'V',V);
      setField(obj.controller_data,'Vdot',Vdot);
%     scope('Atlas','V',t,V,struct('linespec','b','scope_id',1));
%     scope('Atlas','Vdot',t,Vdot,struct('linespec','g','scope_id',1));
    else
      setField(obj.controller_data,'V',0);
      setField(obj.controller_data,'Vdot',0);
    end
    
    if (obj.use_mex==2)
      if ctrl_data.ignore_terrain
        contact_threshold =-1;       
      end
      if obj.using_flat_terrain
        height = getTerrainHeight(r,[0;0]); % get height from DRCFlatTerrainMap
      else
        height = 0;
      end
      [y,Vdotmex,active_supports_mex,Q,gobj,A,rhs,sense,lb,ub] = QPControllermex(obj.mex_ptr.getData(),q_ddot_des,x,desired_supports,A_ls,B_ls,Qy,R_ls,C_ls,D_ls,S,s1,x0,u0,y0,mu,rfoot_contact_state,lfoot_contact_state,contact_threshold,height);
      valuecheck(active_supports_mex,active_supports);
      valuecheck(Q'+Q,model.Q'+model.Q);
      valuecheck(gobj,model.obj);
      valuecheck(A,model.A);
      valuecheck(rhs,model.rhs);
      valuecheck(sense',model.sense);
      valuecheck(lb,model.lb);
      valuecheck(ub,model.ub);
%       valuecheck(y,des.y,1e-4);  % they are close, but not *quite* the
%       same. ---I don't like this, I'm seeing differences up to 5Nm in
%       some dimensions.
%      valuecheck(Vdotmex,Vdot,1e-4);  % this one, too. (coincidentally
%      Vdotmex was < Vdot in the very few cases I looked at carefully)
    end
    
   
    if obj.debug && (obj.use_mex==0 || obj.use_mex==2) && nc > 0
%       if nq_free > 0
%         xcomdd = Jdot * qd + J * qdd;
%       else
%         xcomdd = Jdot * qd + J * alpha(1:nq);
%       end
%       zmppos = xcom(1:2) + D_ls * xcomdd;
%       convh = convhull(cpos(1,:), cpos(2,:));
%       zmp_ok = inpolygon(zmppos(1), zmppos(2), cpos(1,convh), cpos(2,convh));
%       if zmp_ok
%         color = [0 1 0];
%       else
%         color = [1 0 0];
%       end
%       plot_lcm_points([zmppos', mean(cpos(3,:))], color, 660, 'Commanded ZMP', 1, true);

%       foot_dot = Jp*qd(obj.con_dof);
%       scope('Atlas','lfoot_dot_x',t,foot_dot(1),struct('linespec','r','scope_id',1));
%       scope('Atlas','lfoot_dot_y',t,foot_dot(2),struct('linespec','g','scope_id',1));
%       scope('Atlas','lfoot_dot_z',t,foot_dot(3),struct('linespec','b','scope_id',1));
% 
%       ffoot_dot = Jp*qd_filt(obj.con_dof);
%       scope('Atlas','lfoot_dot_fx',t,ffoot_dot(1),struct('linespec','r','scope_id',2));
%       scope('Atlas','lfoot_dot_fy',t,ffoot_dot(2),struct('linespec','g','scope_id',2));
%       scope('Atlas','lfoot_dot_fz',t,ffoot_dot(3),struct('linespec','b','scope_id',2));
      

      state_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      lax = find(~cellfun(@isempty,strfind(state_names,'l_leg_lax')));
      uay = find(~cellfun(@isempty,strfind(state_names,'l_leg_uay')));

      scope('Atlas','lax_qdd_des',t,q_ddot_des(lax),struct('linespec','r','scope_id',1));
      scope('Atlas','uay_qdd_des',t,q_ddot_des(uay),struct('linespec','b','scope_id',1));
      scope('Atlas','lax_qdd',t,qdd(lax),struct('linespec','r','scope_id',2));
      scope('Atlas','uay_qdd',t,qdd(uay),struct('linespec','b','scope_id',2));
      
%       m = drc.controller_zmp_status_t();
%       m.utime = t * 1e6;
%       m.zmp_ok = zmp_ok;
%       obj.lc.publish('CONTROLLER_ZMP_STATUS', m);
      
      [~,normals] = getTerrainHeight(r,cpos);
      d = RigidBodyManipulator.surfaceTangents(normals);

      lambda = Iz*alpha;
      beta_full = Ibeta*alpha;
      for kk=1:8
        if kk<=nc
          plot_lcm_points([cpos(:,kk) cpos(:,kk)+0.25*normals(:,kk)]', [0 0 1; 0 0 1], 23489083+kk, sprintf('Foot Contact Normal %d',kk), 2, true);
          beta = beta_full((kk-1)*nd+(1:nd),:);
          fvec = lambda(kk)*normals(:,kk) + d{1}(:,kk)*beta(1) + d{2}(:,kk)*beta(2) - d{1}(:,kk)*beta(3) - d{2}(:,kk)*beta(4);
          plot_lcm_points([cpos(:,kk) cpos(:,kk)+0.0025*fvec]', [1 0 0; 1 0 0], 6643+kk, sprintf('Foot Contact Force %d',kk), 2, true);
        else
          plot_lcm_points(zeros(2,3), [0 0 1;0 0 1], 23489083+kk, sprintf('Foot Contact Normal %d',kk), 2, true);
          plot_lcm_points(zeros(2,3), [1 0 0;1 0 0], 6643+kk, sprintf('Foot Contact Force %d',kk), 2, true);
        end
      end

%       % plot body coordinate frames
%       m=vs.obj_collection_t();
%       m.objs = javaArray('vs.obj_t', size(1, 1));
%       m.id=13300;
%       m.type=5; % rgb triad
%       m.name='Drake Body Coords';
%       m.reset=true;
%       m.nobjs=5; 
%       
%       pelvis = findLinkInd(r,'pelvis');
%       xzyrpy = forwardKin(r,kinsol,pelvis,[0;0;0],1);
%       msg=vs.obj_t();
%       msg.id=1;
%       msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
%       msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
%       m.objs(msg.id) = msg;
% 
%       head = findLinkInd(r,'head');
%       xzyrpy = forwardKin(r,kinsol,head,[0;0;0],1);
%       msg=vs.obj_t();
%       msg.id=2;
%       msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
%       msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
%       m.objs(msg.id) = msg;
% 
%       xzyrpy = forwardKin(r,kinsol,obj.rfoot_idx,[0;0;0],1);
%       msg=vs.obj_t();
%       msg.id=3;
%       msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
%       msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
%       m.objs(msg.id) = msg;
% 
%       xzyrpy = forwardKin(r,kinsol,obj.lfoot_idx,[0;0;0],1);
%       msg=vs.obj_t();
%       msg.id=4;
%       msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
%       msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
%       m.objs(msg.id) = msg;
% 
%       xzyrpy = x(1:6); 
%       msg=vs.obj_t();
%       msg.id=5;
%       msg.x=xzyrpy(1); msg.y=xzyrpy(2); msg.z=xzyrpy(3);
%       msg.roll=xzyrpy(4); msg.pitch=xzyrpy(5); msg.yaw=xzyrpy(6);
%       m.objs(msg.id) = msg;
%       
%       obj.lc.publish('OBJ_COLLECTION', m);
    end

    if (0)     % simple timekeeping for performance optimization
      % note: also need to uncomment tic at very top of this method
      out_toc=toc(out_tic);
      persistent average_tictoc average_tictoc_n;
      if isempty(average_tictoc)
        average_tictoc = out_toc;
        average_tictoc_n = 1;
      else
        average_tictoc = (average_tictoc_n*average_tictoc + out_toc)/(average_tictoc_n+1);
        average_tictoc_n = average_tictoc_n+1;
      end
      if mod(average_tictoc_n,50)==0
        fprintf('Average control output duration: %2.4f\n',average_tictoc);
      end
    end
  end
  end

  properties (SetAccess=private)
    robot; % to be controlled
    controller_data; % shared data handle that holds S, h, foot trajectories, etc.
    w; % objective function weight
    slack_limit; % maximum absolute magnitude of acceleration slack variable values
    free_dof % dofs for which we perform unconstrained minimization
    con_dof 
    free_inputs
    con_inputs
    rfoot_idx;
    lfoot_idx;
    rhand_idx;
    lhand_idx;
    R; % quadratic input cost matrix
    solver = 0; % 0: gurobi, 1:cplex
    solver_options = struct();
    debug;
    use_mex;
    use_hand_ft;
    mex_ptr;
    lc;
    contact_est_monitor;
    lcm_foot_contacts;  
    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
    num_body_contacts; % vector of num contacts for each body
    multi_robot;
    num_state_frames; % if there's a multi robot defined this is 1+ the number of other state frames
    using_flat_terrain; % true if using DRCFlatTerrain
  end
end
