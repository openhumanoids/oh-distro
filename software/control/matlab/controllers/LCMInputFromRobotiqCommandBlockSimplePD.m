classdef LCMInputFromRobotiqCommandBlockSimplePD < MIMODrakeSystem
  
  properties
    lc; % LCM
    lcmonitor; %LCM monitor
    lcmtype_constructor;
    coder;
    lcmtype;
    coordinate_names;
    timestamp_name;
    dim;
    % Atlas for referencing hand input:
    r;
    % Target finger positions
    target_positions = [0;0;0];
    % Their position, velocity indices
    in_indices;
    % Output order
    out_indices;

    pp;
  end
  
  methods
    function obj = LCMInputFromRobotiqCommandBlockSimplePD(r, options)
      typecheck(r,{'Atlas', 'IRB140'});

      output_frame = r.getInputFrame().getFrameByName('HandInput');
      input_frame = r.getOutputFrame().getFrameByName('HandState');

      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      obj.r = r;
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmonitor = drake.util.MessageMonitor(robotiqhand.command_t,'utime');
      obj.lc.subscribe('ROBOTIQ_RIGHT_COMMAND',obj.lcmonitor); %this should be an arg when / if we move to 2 hands
      
      lcmtype = robotiqhand.command_t;
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
      q_closed = [1.2427, 1.0864, -0.1336, 1.2427, 1.0864,-0.1336, 1.2427, 0.9114, 0.0000]';
      q_open = zeros(numel(q_closed), 1);
      obj.pp = spline([0, 1], [q_open, q_closed]);

      % Precompute indices for our position control
      obj.in_indices = [obj.getInputFrame.findCoordinateIndex('finger_middle_joint_0');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_1');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_2');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_0');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_1');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_2');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_0');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_1');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_2');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_0dot');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_1dot');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_2dot');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_0dot');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_1dot');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_2dot');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_0dot');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_1dot');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_2dot')];
      obj.out_indices = [obj.getOutputFrame.findCoordinateIndex('finger_middle_joint_0');
                        obj.getOutputFrame.findCoordinateIndex('finger_middle_joint_1');
                        obj.getOutputFrame.findCoordinateIndex('finger_middle_joint_2');
                        obj.getOutputFrame.findCoordinateIndex('finger_1_joint_0');
                        obj.getOutputFrame.findCoordinateIndex('finger_1_joint_1');
                        obj.getOutputFrame.findCoordinateIndex('finger_1_joint_2');
                        obj.getOutputFrame.findCoordinateIndex('finger_2_joint_0');
                        obj.getOutputFrame.findCoordinateIndex('finger_2_joint_1');
                        obj.getOutputFrame.findCoordinateIndex('finger_2_joint_2')];
    end
    
    function x=decode(obj, data)
      msg = obj.lcmtype_constructor.newInstance(data);
      for i=1:obj.dim
        eval(['x.',obj.coordinate_names{i},' = msg.',CoordinateFrame.stripSpecialChars(obj.coordinate_names{i}),';']);
      end
      eval(['t = msg.', obj.timestamp_name, '/1000;']);
    end
    
    function varargout=mimoOutput(obj,t,~,hand_state)
      
      % What needs to go out:
      efforts = zeros(obj.getNumOutputs, 1);
      
      % see if we have a new message (new command state)
      data = obj.lcmonitor.getMessage();
        
      % If we haven't received a command, apply outward force
      if (~isempty(data))
        cmd = obj.decode(data);
        % Use this to set our goal state
        if cmd.mode==0
          obj.target_positions(:) = (cmd.position/255.0);
        else
          % Don't know how to handle this yet
          fprintf('Nonzero hand control mode\n')
        end
        
      end
      
      % pd position control
      % I should vectorize this...
      pos = hand_state(obj.in_indices(1:9));
      vel = hand_state(obj.in_indices(10:18));
      
      kp = -3;
      kd = 0.1;

      q_desired = ppval(obj.pp, max(obj.target_positions));

      efforts(obj.out_indices) = kp*(pos-q_desired(obj.in_indices(1:9))) - kd*vel;
      
      varargout = {efforts};
    end
  end
  
end
