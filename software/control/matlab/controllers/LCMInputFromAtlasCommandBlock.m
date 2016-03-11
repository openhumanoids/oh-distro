classdef LCMInputFromAtlasCommandBlock < MIMODrakeSystem
  
  properties
    lc;
    lcmonitor_cmd; %LCM monitors
    lcmonitor_neck;
    
    lcmtype_cmd_constructor;
    lcmtype_neck_constructor;
    coder_cmd;
    lcmtype_cmd;
    lcmtype_neck;
    
    lcmtype_cmd_coordinate_names;
    lcmtype_cmd_dim;
    timestamp_name;
    
    % Atlas and various controllers:
    r;
    r_control;
    plan_eval_and_control
    nq; % Atlas # of DOFS
    nu; % Atlas # of controllable DOFS
    joint_names;
    drake_to_atlas_joint_map;
    neck_in_i;
    neck_out_i;
    neck_desired_angle;
    
    %
    l_foot_id;
    r_foot_id;
  end
  
  methods
    function obj = LCMInputFromAtlasCommandBlock(r, r_control, options)
      typecheck(r,'Atlas');
      % r_control is an atlas model with a state of just
      % atlas_state.
      if ~isempty(r_control)
        typecheck(r_control, 'Atlas');
      else
        r_control = r;
      end
      
      if nargin<2
        options = struct();
      end

      % Generate AtlasInput as out (we'll do translation manually)
      output_frame = drcFrames.AtlasInput(r);
      
      % We'll need atlas state as input
      input_frame = drcFrames.AtlasState(r_control);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      obj.r = r;
      obj.r_control = r_control;
      obj.l_foot_id = obj.r_control.findLinkId('l_foot');
      obj.r_foot_id = obj.r_control.findLinkId('r_foot');
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmonitor_cmd = drake.util.MessageMonitor(bot_core.atlas_command_t,'utime');
      obj.lcmonitor_neck = drake.util.MessageMonitor(drc.neck_pitch_t,'utime');
      obj.lc.subscribe('ATLAS_COMMAND',obj.lcmonitor_cmd);
      obj.lc.subscribe('DESIRED_NECK_PITCH',obj.lcmonitor_neck);
      
      % Setup ATLAS_COMMAND_T lcm type
      lcmtype_cmd = bot_core.atlas_command_t;
      lcmtype_cmd = lcmtype_cmd.getClass();
      names={};
      f = lcmtype_cmd.getFields;
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
      obj.lcmtype_cmd = lcmtype_cmd;
      obj.lcmtype_cmd_coordinate_names = names;
      obj.lcmtype_cmd_dim = length(names);
      constructors_cmd = lcmtype_cmd.getConstructors();
      for i=1:length(constructors_cmd)
        f = constructors_cmd(i).getParameterTypes;
        if ~isempty(f) && strncmp('[B',char(f(1).getName),2)
          obj.lcmtype_cmd_constructor = constructors_cmd(i);
        end
      end
      
      % Setup NECK_PITCH_T lcm type
      lcmtype_neck = drc.neck_pitch_t;
      lcmtype_neck = lcmtype_neck.getClass();
      constructors_neck = lcmtype_neck.getConstructors();
      for i=1:length(constructors_neck)
        f = constructors_neck(i).getParameterTypes;
        if ~isempty(f) && strncmp('[B',char(f(1).getName),2)
          obj.lcmtype_neck_constructor = constructors_neck(i);
        end
      end
      
      % And the initial standing controller, until the planner comes
      % online
      obj = setup_init_planner(obj, options);
      
      % And the lcm coder
      obj.joint_names = obj.r_control.getInputFrame().getCoordinateNames();
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
      obj.coder_cmd = drc.control.AtlasCommandCoder(obj.joint_names,r.atlas_version,gains.k_q_p*0,gains.k_q_i*0,...
        gains.k_qd_p,gains.k_f_p*0,gains.ff_qd,gains.ff_qd_d,gains.ff_f_d*0,gains.ff_const*0);
      
      % And compute for ourselves the drake_to_atlas_joint_map
      obj.drake_to_atlas_joint_map = zeros(length(obj.joint_names), 1);
      dummyInput = drcFrames.AtlasInput(obj.r);
      for i=1:length(obj.joint_names)
        in_joint_name_i = obj.joint_names(i, :);
        obj.drake_to_atlas_joint_map(i) = dummyInput.findCoordinateIndex(strtrim(in_joint_name_i));
      end

      % Set up neck joint indices
      dummyInput = drcFrames.AtlasInput(obj.r_control);
      dummyState = drcFrames.AtlasState(obj.r_control);
      obj.neck_in_i = dummyState.findCoordinateIndex('neck_ay');
      obj.neck_in_i = obj.neck_in_i(1);
      obj.neck_out_i = dummyInput.findCoordinateIndex('neck_ay');
      obj.neck_desired_angle = SharedDataHandle(0);
      
    end
    
    function obj=setup_init_planner(obj, options)

      r = obj.r_control;

      S = load(r.fixed_point_file);
      x_init = Point(r.getStateFrame(),getInitialState(r));
      xstar = Point(r.getStateFrame()); 
      xstar(1:length(S.xstar)) = S.xstar;
      xstar = Point(r.getStateFrame(), xstar);
      xstar.base_x = x_init.base_x;
      xstar.base_y = x_init.base_y;
      xstar.base_z = x_init.base_z;
      xstar.base_yaw = x_init.base_yaw;
      
      x0 = double(xstar);
      obj.nq = getNumPositions(r);
      obj.nu = getNumInputs(r);

      standing_plan = QPLocomotionPlanCPPWrapper(QPLocomotionPlanSettings.fromStandingState(x0, r));
      %standing_plan.planned_support_command = QPControllerPlan.support_logic_maps.kinematic_or_sensed;
      plan_eval = bipedControllers.BipedPlanEval(r, standing_plan);
      control = bipedControllers.InstantaneousQPController(r);
      obj.plan_eval_and_control = bipedControllers.BipedPlanEvalAndControlSystem(r, control, plan_eval);
    end
    
    function x=decode_cmd(obj, data)
      msg = obj.lcmtype_cmd_constructor.newInstance(data);
      x=cell(obj.dim, 1);
      for i=1:obj.dim
        eval(['x{',num2str(i),'} = msg.',CoordinateFrame.stripSpecialChars(obj.lcmtype_cmd_coordinate_names{i}),';']);
        % fix string type
        if (isa(x{i}, 'java.lang.String[]'))
          x{i} = char(x{i});
        end
      end
      eval(['t = msg.', obj.timestamp_name, '/1000;']);
    end
    
    function varargout=mimoOutput(obj,t,~,atlas_state)
      
      % What needs to go out:
      %atlas_state_names = obj.getInputFrame.getCoordinateNames();
      efforts = zeros(obj.getNumOutputs, 1);
      
      % check for new neck cmd
      data_neck = obj.lcmonitor_neck.getMessage();
      if (~isempty(data_neck))
       neck_cmd = obj.lcmtype_neck_constructor.newInstance(data_neck);
       obj.neck_desired_angle.setData(neck_cmd.pitch);
      end
      
      % see if we have a new message (new command state)
      data = obj.lcmonitor_cmd.getMessage();
        
      % If we haven't received a command make our own
      if (isempty(data))
        y = obj.plan_eval_and_control.output(t, [], atlas_state);

        efforts(obj.drake_to_atlas_joint_map(1:obj.nu)) = y(1:obj.nu);
      else
        cmd = obj.coder_cmd.decode(data);
        efforts = cmd.val(obj.nu*2+1:end);
      end
      
      % And set neck pitch via simple PD controller
      
      error =  obj.neck_desired_angle.getData - atlas_state(obj.neck_in_i);
      vel = atlas_state(length(atlas_state)/2 + obj.neck_in_i);
      efforts(obj.neck_out_i) = 50*error - vel;
      varargout = {efforts(1:obj.nu)};
      %fprintf('\b\b\b\b\b\b\b%7.3f', t);
      
    end
  end
  
end
