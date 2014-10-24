classdef LCMInputFromAtlasCommandBlock < MIMODrakeSystem
  
  properties
    lc; % LCM
    lcmonitor; %LCM monitor
    lcmtype_constructor;
    coder;
    lcmtype;
    coordinate_names;
    timestamp_name;
    dim;
    % Atlas and various controllers:
    robot;
    pelvis_controller;
    fc;
    qt;
    pd_plus_qp_block;
    nq; % Atlas # of DOFS
    nu; % Atlas # of controllable DOFS
    joint_names;
    drake_to_atlas_joint_map;
  end
  
  methods
    function obj = LCMInputFromAtlasCommandBlock(r,options)
      typecheck(r,'Biped');
      
      if nargin<2
        options = struct();
      end

      % Generate AtlasInput as out (we'll do translation manually)
      output_frame = getInputFrame(r);
      
      % We'll need atlas state as input
      input_frame = getStateFrame(r);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmonitor = drake.util.MessageMonitor(drc.atlas_command_t,'utime');
      obj.lc.subscribe('ATLAS_COMMAND',obj.lcmonitor);
      
      lcmtype = drc.atlas_command_t;
      lcmtype = lcmtype.getClass();
      names={};
      f = lcmtype.getFields;
      for i=1:length(f)
        fname = char(f(i).getName());
        if strncmp(fname,'LCM_FINGERPRINT',15), continue; end
        if strcmp(fname,'utime')
          if ~strcmp(f(i).getGenericType.getName,'long')
            error('by convention, the timestamp field should be type int64_t');
          end
          obj.timestamp_name = 'utime';
          continue;
        end
        names{end+1}=fname;
      end
      obj.lcmtype = lcmtype;
      obj.coordinate_names = names;
      obj.dim = length(names);
      constructors = lcmtype.getConstructors();
      for i=1:length(constructors)
        f = constructors(i).getParameterTypes;
        if ~isempty(f) && strncmp('[B',char(f(1).getName),2)
          obj.lcmtype_constructor = constructors(i);
        end
      end
      
      % And the initial standing controller, until the planner comes
      % online
      obj = setup_init_planner(obj, r, options);
      
      % And the lcm coder
      obj.joint_names = obj.pd_plus_qp_block.getOutputFrame.getCoordinateNames;
      obj.joint_names = obj.joint_names(1:obj.nu);
      gains = struct();
      gains.k_qd_p = zeros(obj.nu,1);
      gains.k_q_i = zeros(obj.nu,1);
      gains.k_f_p = zeros(obj.nu,1);
      gains.ff_f_d = zeros(obj.nu,1);
      gains.ff_qd_d = zeros(obj.nu,1);
      gains.ff_const = zeros(obj.nu,1);
      [Kp,Kd] = getPDGains(r,'default');
      gains.k_q_p = diag(Kp);
      gains.ff_qd = diag(Kd);
      obj.coder = drc.control.AtlasCommandCoder(obj.joint_names,gains.k_q_p*0,gains.k_q_i*0,...
        gains.k_qd_p,gains.k_f_p*0,gains.ff_qd,gains.ff_qd_d,gains.ff_f_d*0,gains.ff_const*0);
      
      % And compute for ourselves the drake_to_atlas_joint_map
      obj.drake_to_atlas_joint_map = zeros(length(obj.joint_names), 1);
      for i=1:length(obj.joint_names)
        in_joint_name_i = obj.joint_names(i, :);
        obj.drake_to_atlas_joint_map(i) = obj.getOutputFrame.findCoordinateIndex(strtrim(in_joint_name_i));
      end

    end
    
    function obj=setup_init_planner(obj, r, options)
      % set initial state to fixed point
      load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      r = r.setInitialState(xstar);
      
      x0 = xstar;
      obj.nq = getNumPositions(r);
      obj.nu = getNumInputs(r);
      q0 = x0(1:obj.nq);
      kinsol = doKinematics(r,q0);
      
      com = getCOM(r,kinsol);
      
      % build TI-ZMP controller
      footidx = [findLinkInd(r,'l_foot'), findLinkInd(r,'r_foot')];
      foot_pos = terrainContactPositions(r,kinsol,footidx);
      comgoal = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';
      limp = LinearInvertedPendulum(com(3));
      [~,V] = lqr(limp,comgoal);
      
      foot_support = RigidBodySupportState(r,find(~cellfun(@isempty,strfind(r.getLinkNames(),'foot'))));
      
      pelvis_idx = findLinkInd(r,'pelvis');
      
      link_constraints(1).link_ndx = pelvis_idx;
      link_constraints(1).pt = [0;0;0];
      link_constraints(1).traj = ConstantTrajectory(forwardKin(r,kinsol,pelvis_idx,[0;0;0],1));
      link_constraints(2).link_ndx = footidx(1);
      link_constraints(2).pt = [0;0;0];
      link_constraints(2).traj = ConstantTrajectory(forwardKin(r,kinsol,footidx(1),[0;0;0],1));
      link_constraints(3).link_ndx = footidx(2);
      link_constraints(3).pt = [0;0;0];
      link_constraints(3).traj = ConstantTrajectory(forwardKin(r,kinsol,footidx(2),[0;0;0],1));
      
      
      ctrl_data = QPControllerData(false,struct(...
        'acceleration_input_frame',AtlasCoordinates(r),...
        'D',-com(3)/9.81*eye(2),...
        'Qy',eye(2),...
        'S',V.S,...
        's1',zeros(4,1),...
        's2',0,...
        'x0',[comgoal;0;0],...
        'u0',zeros(2,1),...
        'y0',comgoal,...
        'qtraj',x0(1:obj.nq),...
        'support_times',0,...
        'supports',foot_support,...
        'link_constraints',link_constraints,...
        'mu',1.0,...
        'ignore_terrain',false,...
        'plan_shift',zeros(3,1),...
        'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'back');findJointIndices(r,'neck')]));
      
      % instantiate QP controller
      options.Kp_pelvis = [150; 150; 150; 200; 200; 200];
      options.pelvis_damping_ratio = 0.6;
      options.Kp_q = 150.0*ones(r.getNumPositions(),1);
      options.q_damping_ratio = 0.6;
      
      % construct QP controller and related control blocks
      [qp,~,~,pelvis_controller,pd,options] = constructQPBalancingController(r,ctrl_data,options);
      obj.pelvis_controller = pelvis_controller;
      options.use_lcm=false;
      options.contact_threshold = 0.002;
      obj.fc = FootContactBlock(r,ctrl_data,options);
      obj.qt = QTrajEvalBlock(r,ctrl_data,options);
      
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 1;
      ins(2).input = 2;
      ins(3).system = 2;
      ins(3).input = 1;
      ins(4).system = 2;
      ins(4).input = 3;
      ins(5).system = 2;
      ins(5).input = 4;
      outs(1).system = 2;
      outs(1).output = 1;
      outs(2).system = 2;
      outs(2).output = 2;
      obj.pd_plus_qp_block = mimoCascade(pd,qp,[],ins,outs);
      clear ins;
      fprintf('Current time: xxx.xxx');
    end
    
    function x=decode(obj, data)
      msg = obj.lcmtype_constructor.newInstance(data);
      x=cell(obj.dim, 1);
      for i=1:obj.dim
        eval(['x{',num2str(i),'} = msg.',CoordinateFrame.stripSpecialChars(obj.coordinate_names{i}),';']);
        % fix string type
        if (isa(x{i}, 'java.lang.String[]'))
          x{i} = char(x{i});
        end
      end
      eval(['t = msg.', obj.timestamp_name, '/1000;']);
    end
    
    function varargout=mimoOutput(obj,t,~,x)
      atlas_state = x;
      %old_output = varargin{2};
      
      % What needs to go out:
      %atlas_state_names = obj.getInputFrame.getCoordinateNames();
      efforts = zeros(obj.getNumOutputs, 1);
      
      % see if we have a new message (new command state)
      data = obj.lcmonitor.getMessage();
        
      % If we haven't received a command make our own
      if (isempty(data))
        % foot contact
        fc = output(obj.fc,t,[],x);
        % qtraj eval
        q_des_and_x = output(obj.qt,t,[],x);
        q_des = q_des_and_x(1:obj.nq);
        % IK/QP
        pelvis_ddot = output(obj.pelvis_controller,t,[],x);
        u_and_qdd = output(obj.pd_plus_qp_block,t,[],[q_des; x; x; fc; pelvis_ddot]);
        u=u_and_qdd(1:obj.nu);
        for i=1:obj.nu
          efforts(obj.drake_to_atlas_joint_map(i)) = u(i);
        end
      else
        cmd = obj.coder.decode(data);
        efforts = cmd.val(obj.nu*2+1:end);
      end
      
      % And set neck pitch via simple PD controller
      neck_in_i = obj.getInputFrame.findCoordinateIndex('neck_ay');
      neck_in_i = neck_in_i(1);
      neck_out_i = obj.getOutputFrame.findCoordinateIndex('neck_ay');
      error =  30*pi/180 - atlas_state(neck_in_i);
      vel = atlas_state(length(atlas_state)/2 + neck_in_i);
      efforts(neck_out_i) = 50*error - vel;
      
      varargout = {efforts};
      fprintf('\b\b\b\b\b\b\b%7.3f', t);
      
    end
  end
  
end
