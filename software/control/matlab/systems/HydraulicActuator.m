classdef HydraulicActuator < DrakeSystem
% Valve spool and cylinder dynamics for a hydraulic actuator 
  
  properties
    %%%% System properties %%%
    p_supply = 3000 * 0.0689; % [bar] system supply pressure
    p_return = 100 * 0.0689; % [bar] system return (tank) pressure
    beta = 13000; % [bar] effective bulk modulus
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%% Valve properties %%%%
    Kspool = 1; % steady-state input-to-spool-position gain
    omega = 200*2*pi; % [rad/sec] spool natural angular frequency 
    xi = 0.5; % damping ratio
    max_input = 10; % [mA] maximum current supplied to valve
    nominal_flow = 0.000113071; % [m^3/s] should be available in valve datasheet 
    nominal_current = 10; % [mA] should be available in valve datasheet 
    nominal_pressure = 68.95; % [bar] should be available in valve datasheet 
    Kv; % valve gain, computed from nominal flow, current, and pressure
    %%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%% Cylinder properties %
    Apos = 0.5891 / 1550; % [m^2] area on the positive side of the cylinder
    Aneg = 0.7854 / 1550; % [m^2]
    lc = 0.1; % [m] length of the cylinder
    Vhose = 3e-5; % [m^3] hose pipeline volume
    Fv = 10; % [Ns/m] viscous damping coefficient    
    mass = 10.0; % [kg] mass of the piston + fluid
    %%%%%%%%%%%%%%%%%%%%%%%%%%
  end
  
  methods
    function obj = HydraulicActuator(params)
      obj = obj@DrakeSystem(6,0,1,7,true,true);
      obj = setInputLimits(obj,-obj.max_input,obj.max_input);
      % TODO: add piston position limit
      
      % compute valve gain      
      obj.Kv = obj.nominal_flow / (obj.nominal_current * sqrt(obj.nominal_pressure/2)); 
    end
    
    function f=dynamics(obj,t,x,u)
      % x = [x_v, dx_v, x_p, dx_p, p_pos, p_neg]

      f=zeros(6,1);
      % second-order spool dynamics
      f(1) = x(2);
      f(2) = obj.omega^2*(obj.Kspool*u - 2*obj.xi/obj.omega * x(2) - x(1));

      % compute flow 
      if x(1) > 0
        qpos = obj.Kv*x(1)*sign(obj.p_supply-x(5))*sqrt(abs(obj.p_supply-x(5)));
        qneg = -obj.Kv*x(1)*sign(x(6)-obj.p_return)*sqrt(abs(x(6)-obj.p_return));
      else
        qpos = obj.Kv*x(1)*sign(x(5)-obj.p_return)*sqrt(abs(x(5)-obj.p_return));
        qneg = -obj.Kv*x(1)*sign(obj.p_supply-x(6))*sqrt(abs(obj.p_supply-x(6)));
      end
      
      % compute chamber volumes
      Vpos = obj.Vhose + obj.Apos*x(3);
      Vneg = obj.Vhose + obj.Aneg*(obj.lc-x(3));
    
      f(3) = x(4);
      Fext = 0;
      f(4) = (obj.Apos*x(5)*1e5 - obj.Aneg*x(6)*1e5 - obj.Fv*x(4) + Fext) / obj.mass;  
      f(5) = obj.beta / Vpos * (qpos - obj.Apos*x(4));
      f(6) = obj.beta / Vneg * (qneg + obj.Aneg*x(4));

      %assert(x(3)>=0 && x(3)<=obj.lc);
    end
    
    function y=output(obj,t,x,u)
      % outputs actuator force
      f_out = obj.Apos*x(5)*1e5 - obj.Aneg*x(6)*1e5 - obj.Fv*x(4);
      y=[x;f_out];
    end
    
    function x = getInitialState(obj)
      p_neg = 0.5*obj.p_return;
      p_pos = obj.Aneg*p_neg/obj.Apos;
      x = [0;0;obj.lc/2;0;p_pos;p_neg];
    end
    
    function val = sg(obj,x)
      val=0;
      if x>=0
        val=x;
      end
    end
  end  
  
end
