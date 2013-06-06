classdef StateCorrupter < DrakeSystem
  
  methods
    function obj = StateCorrupter(robot,params)
      typecheck(robot,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
      obj = obj@DrakeSystem(0,1,getNumStates(robot),getNumStates(robot),true,true);
      obj = setInputFrame(obj,getStateFrame(robot));
      obj = setOutputFrame(obj,getStateFrame(robot));
      obj.robot = robot;
      obj.params = params;
    end
    
    function state = ramp(terms,state)
      state.data = terms.rate*state.t;
    end
    function state = gaussmarkov(terms,state)
      state.data = state.prev + terms.sigma * (terms.dt);
    end
    function state = whitenoise(terms,state)
      state.data = terms.sigma*randn();
    end
    
    function ret = addcorruption(type, terms, prev)
      
      state = struct();
      state.prev = prev;
      state = eval([type '(terms,state)']);
      ret = state.data;
    end
    
    function ret = eval_per_group(t,types,terms,prev)
      if (length(types) ~= length(terms))
        disp('ERROR: StateCorrupter.eval_per_group -- length of types and terms do not match');
      end
        
      for n=1:numel(types)
        state.data = vals;
        state.t = t;
          
        ret = addcorruption(types{n},terms{n},prev);
      end
      
    end
    
    function ret = eval_groups(t,chk,var,prev)
      if (isfield(chk,var))
        if (eval(['isfield(chk.' var ',''types'')']))
          ret = eval_per_group(t, eval(['chk.',var,'''types''']),  eval(['chk.' var '.terms']), vals);
        end
      end
    end
    
    
    function xn = update(obj,t,x,u)
      %use x as state memory
      % x must be double vector type
      %
      % values on x have a fixed ordering:
      %  [pos, angles, joints, vel, rates, jointrates, deltaT]'
      %        6         28        6           28         1
        
      P = zeros(3,1);
      E = zeros(3,1);
      V = zeros(3,1);
      R = zeros(3,1);
      
      memP = x(1:3);
      memE = x(4:6);
      memV = x(7:9);
      memR = x(10:12);
      
      allvals = {'P','V','E','R'};
      
      for n=1:numel(allvals)
        %eval_groups(t,obj.params,'P',P);
        eval([allvals{n},'=eval_groups(t,obj.params,''',allvals{n},''',',allvals{n},',mem',allvals{n},')']);
      end
    end
    
    function y = output(obj,t,x,u)
      % u is the input
      % y is the output at the next time step
      % x is state memory propagated separately in "xn = update(.x.)" above
      
      y = u;
      
      % TODO -- remove when code is ready
      return;
      
      y(1:3) = u(1:3) + x(1:3);
      y(4:6) = u(4:6) + x(4:6);
      y(35:37) = u(35:37) + x(7:9);
      y(38:40) = u(38:40) + x(10:12);
    end
  end
  
  properties
    robot
    params
  end
end