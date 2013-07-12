classdef StateCorrupter < DrakeSystem
  
  methods
    function obj = StateCorrupter(robot,params)
      typecheck(robot,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
      obj = obj@DrakeSystem(0,42,getNumStates(robot),getNumStates(robot),true,true);
      obj = setInputFrame(obj,getStateFrame(robot));
      obj = setOutputFrame(obj,getStateFrame(robot));
      obj.robot = robot;
      obj.params = params;
    end
    
    function state = ramp(state)
      state.data = state.terms.rate*state.t;
    end
    function state = gaussmarkov(state)
      prev = state.mem.data(state.mem.pointer);
      state.data = prev + state.terms.sigma*randn()*0.005; % assuming 200Hz
      state.mem.data(state.mem.pointer) = state.data;
      state.mem.pointer = state.mem.pointer+1;
    end
    function state = whitenoise(state)
      state.data = state.terms.sigma*randn();
    end

    function [ret,mem] = addcorruption(t, type, terms, prev, mem)

      % own type -- starts empty
      state = struct();

      % MUX
      state.mem = mem;
      state.t = t;
      state.terms = terms;

      % eval
      state = eval([type '(state);']);

      % DEMUX
      ret = state.data;
      mem = state.mem;
    end

    function [ret,mem] = eval_per_group(t,types,terms,prev,mem)

      if (length(types.types) ~= length(terms))
        disp('ERROR: StateCorrupter.eval_per_group -- length of types and terms do not match');
      end

      ret = zeros(3,1);

      for n=1:numel(types.types)

        % here we check the individual channels
        chanoptions = 'xyz';
        for k=1:numel(chanoptions)
            if (isempty(strfind(types.channels,chanoptions(k)))~=1)
             addvalue = 0;
             [addvalue,mem] = addcorruption(t, types.types{n},terms{n},prev(k),mem);
             ret(k) = ret(k) + addvalue;
            end
        end

      end % for numel(types)

    end

    function [ret,mem] = parse_groups(t,chk,var,prev,mem)

      ret = zeros(3,1);

      if (isfield(chk,var))
        if (eval(['isfield(chk.' var ',''types'')']))


          if (eval(['isfield(chk.',var,',''channels'')'])==0)
            eval(['chk.',var,'.channels = ''xyz'';']);
          end

          eval(['chk.',var,'.int_types.types = chk.',var,'.types;']);
          eval(['chk.',var,'.int_types.channels = chk.',var,'.channels;']);

          [ret,mem] = eval_per_group(t, eval(['chk.',var,'.int_types']),  eval(['chk.' var '.terms']), prev, mem);
        end
      end
    end
    
    function xn = update(obj,t,x,u)
      %use x as state memory

      %The added noise at the previous time step is available as 'prev'
      prev.P = x(1:3);
      prev.E = x(4:6);
      prev.V = x(7:9);
      prev.R = x(10:12);

      % grab prevoius memory elements
      mem.data = x(13:end);
      mem.pointer = 1;

      allvals = {'P','E','V','R'};

      for n=1:numel(allvals)
         eval(['[',allvals{n},',mem]=parse_groups(t,obj.params,''',allvals{n},''',','prev.',allvals{n},',mem);']);
      end

      xn = zeros(length(x),1);
      xn(1:3) = P;
      xn(4:6) = E;
      xn(7:9) = V;
      xn(10:12) = R;

      % store user mempry elements for the next iteration
      xn(13:end) = mem.data;

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