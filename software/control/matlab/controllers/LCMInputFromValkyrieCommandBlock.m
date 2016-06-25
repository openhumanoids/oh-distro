classdef LCMInputFromValkyrieCommandBlock < MIMODrakeSystem
  
  properties
    lc;
    lcmonitor_cmd; %LCM monitors
    
    lcmtype_cmd_constructor
    coder_cmd;
    lcmtype_cmd;
    
    lcmtype_cmd_coordinate_names;
    lcmtype_cmd_dim;
    timestamp_name;
    
    % robot and various controllers:
    r;
    r_control;
    plan_eval_and_control
    nq;
    nu;
    joint_names;
    
    drake_to_robot_joint_map;
    
    %
    l_foot_id;
    r_foot_id;
  end
  
  methods
    function obj = LCMInputFromValkyrieCommandBlock(r, r_control, options)
      typecheck(r,'Valkyrie');
      % r_control is an atlas model with a state of just
      % val state
      if ~isempty(r_control)
        typecheck(r_control, 'Valkyrie');
      else
        r_control = r;
      end
      
      if nargin<2
        options = struct();
      end

      % Generate Input as out (we'll do translation manually)
      output_frame = drcFrames.ValkyrieInput(r);
      
      % We'll need state as input
      input_frame = drcFrames.ValkyrieState(r_control);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      obj.r = r;
      obj.r_control = r_control;
      obj.l_foot_id = obj.r_control.findLinkId('leftFoot');
      obj.r_foot_id = obj.r_control.findLinkId('rightFoot');
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmonitor_cmd = drake.util.MessageMonitor(bot_core.atlas_command_t,'utime');
      obj.lc.subscribe('ROBOT_COMMAND',obj.lcmonitor_cmd);
      
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
      
      % And the initial standing controller, until the planner comes
      % online
      obj = setup_init_planner(obj, options);
      
      % And the lcm coder
      obj.joint_names = obj.r_control.getInputFrame.getCoordinateNames;
      for k=1:length(obj.joint_names)
         l = strfind(obj.joint_names{k}, 'Actuator');
         if (l)
             obj.joint_names{k} = obj.joint_names{k}(1:(l-1));
         end
         l = strfind(obj.joint_names{k}, 'waist');
         if (l)
             obj.joint_names{k} = ['torso', obj.joint_names{k}(6:end)];
         end
      end
      gains = struct();
      gains.k_qd_p = zeros(obj.nu,1);
      gains.k_q_i = zeros(obj.nu,1);
      gains.k_f_p = zeros(obj.nu,1);
      gains.ff_f_d = zeros(obj.nu,1);
      gains.ff_qd_d = zeros(obj.nu,1);
      gains.ff_const = zeros(obj.nu,1);
      gains.k_q_p = zeros(obj.nu, 1); 
      gains.ff_qd = zeros(obj.nu, 1);
      obj.coder_cmd = drc.control.ValkyrieCommandCoder(obj.joint_names,gains.k_q_p*0,gains.k_q_i*0,...
        gains.k_qd_p,gains.k_f_p*0,gains.ff_qd,gains.ff_qd_d,gains.ff_f_d*0,gains.ff_const*0);
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
      
      control = bipedControllers.InstantaneousQPController(r.getManipulator().urdf{1}, r.control_config_file);
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
    
    function varargout=mimoOutput(obj,t,~,robot_state)
      
      % What needs to go out:
      efforts = zeros(obj.getNumOutputs, 1);
      
      % see if we have a new message (new command state)
      data = obj.lcmonitor_cmd.getMessage();
        
      % If we haven't received a command make our own
      if (isempty(data))
        y = obj.plan_eval_and_control.output(t, [], robot_state);

        efforts(1:obj.nu) = y(1:obj.nu);
      else
        cmd = obj.coder_cmd.decode(data);
        efforts = cmd.val(obj.nu*2+1:end);
      end
      
      varargout = {efforts(1:obj.nu)};
    end
  end
  
end
