classdef DelayBlock < DrakeSystem
  % simple DT fixed delay block
  methods
    function obj = DelayBlock(robot,options)
      typecheck(robot,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
      if nargin > 1
        assert(isa(options,'struct'));
      else
        options = struct();
      end
      
      if isfield(options,'use_input_frame')
        % if true, delay block is between controller and robot
        % if false, delay block is between robot and controller
        use_input_frame = options.use_input_frame;  
      else
        use_input_frame = false;  
      end
      
      if isfield(options,'delay_steps')
        delay_steps = options.delay_steps;  
      else
        delay_steps = 0;  
      end

      if isfield(options,'dt')
        dt = options.dt;  
      else
        dt = 0.004;  
      end
      
      if use_input_frame
        nd = getNumInputs(robot);
      else
        nd = getNumStates(robot);
      end
      
      obj = obj@DrakeSystem(0,(delay_steps+1)*nd,nd,nd,true,true);
      obj = setSampleTime(obj,[dt;0]); % sets update rate

      if use_input_frame
        obj = setInputFrame(obj,getInputFrame(robot));
        obj = setOutputFrame(obj,getInputFrame(robot));
      else
        obj = setInputFrame(obj,getStateFrame(robot));
        obj = setOutputFrame(obj,getStateFrame(robot));
      end
      
      obj.delay_steps = delay_steps;
      obj.nd = nd;
    end
    
    function xn = update(obj,t,x,u)
      % use x as queue
      nx = length(x);
      xn = 0*x;

      % shift history up
      xn(1:nx-obj.nd) = x(obj.nd+(1:obj.nd*obj.delay_steps));
      xn(nx-obj.nd+1:end) = u;
    end
    
    function y = output(obj,t,x,u)
      y = x(1:obj.nd); % grab oldest state in queue
    end
  end
  
  properties
    delay_steps;
    nd;
  end
end