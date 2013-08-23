classdef FrictionModel < DrakeSystem
  % Friction model for combination Coulomb, Stribeck, and viscious friction
  % for a system with n joints.
  % For parameter vector g, velocity qd, and desired accel a
  % v = qd + g(7)*a, where the last term is included if use_accel_lookahead
  % is true
  % 
  % F_friction = g(1)*(tanh(g(2)*v) - tanh(g(3)*v))  % stribeck
  %              + g(4)*tanh(g(5)*v)                 % coulomb
  %              + g(6)*v                            % viscous
  methods
    function obj=FrictionModel(friction_params,use_accel_lookahead)
      n = size(friction_params,2);
      if nargin < 2
        use_accel_lookahead = false;
      end
      
      % Awkwardly adding use_accel_lookahead since matlab doesn't allow an
      % if/else to start off a constructor
      obj = obj@DrakeSystem(0,0,(1+use_accel_lookahead)*n,n,false,true);
      
      if use_accel_lookahead
        valuecheck(size(friction_params,1),7);
      else
        valuecheck(size(friction_params,1),6);
      end
      obj.params = friction_params;
      obj.use_accel_lookahead = use_accel_lookahead;
      obj.n = n;
    end
    
    function F_friction = output(obj,t,x,u)
      if obj.use_accel_lookahead
        v = u(1:obj.n) + obj.params(7,:)'.*u(obj.n+1:end);
      else
        v = u;
      end
      F_friction = obj.params(1,:)'.*(tanh(obj.params(2,:)'.*v) - tanh(obj.params(3,:)'.*v)) + ... % stribeck
        + obj.params(4,:)'.*tanh(obj.params(5,:)'.*v) + ...                % coulomb
        + obj.params(6,:)'.*v;                            % viscous
    end
  end
  
  properties
    n
    use_accel_lookahead
    params
  end
end