classdef DeadbandBlock < DrakeSystem
  % simple input deadband block. input in a specified range about zero is
  % simply canceled out
  methods
    function obj = DeadbandBlock(robot,options)
      typecheck(robot,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
      if nargin > 1
        assert(isa(options,'struct'));
      else
        options = struct();
      end
      
      if isfield(options,'dt')
        dt = options.dt;  
      else
        dt = 0.004;  
      end
      
      nd = getNumInputs(robot);
      
      obj = obj@DrakeSystem(0,0,nd,nd,true,true);
      obj = setSampleTime(obj,[dt;0]); % sets update rate
      obj = setInputFrame(obj,getInputFrame(robot));
      obj = setOutputFrame(obj,getInputFrame(robot));
      
      if isfield(options,'deadband')
        typecheck(options.deadband,'double');
        obj.deadband = options.deadband;  
      else
        obj.deadband = 5;  
      end
    end
        
    function y = output(obj,~,~,u)
      y = u;
      y(u<obj.deadband & u>-obj.deadband) = 0;
    end
  end
  
  properties
    deadband;
  end
end