classdef HydraulicActuator < DrakeSystem
% Valve spool and cylinder dynamics for a hydraulic actuator 
  
  properties
    %%%% System properties %%%
    p_supply = 3000 * 0.00689475729; % [MPa] system supply pressure
    p_return = 100 * 0.00689475729; % [MPa] system return (tank) pressure
    beta = 1200; % [MPa] effective bulk modulus
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%% Valve properties %%%%
    Kspool = 1; % steady-state input-to-spool-position gain
    omega = 200*2*pi; % [rad/sec] spool natural angular frequency 
    xi = 0.5; % damping ratio
    max_input = 10; % [mA] maximum current supplied to valve
    nominal_flow = 0.000113071; % [m^3/s] should be available in valve datasheet 
    nominal_current = 10; % [mA] should be available in valve datasheet 
    nominal_pressure = 1000 * 0.00689475729; % [MPa] should be available in valve datasheet 
    Kv; % valve gain, computed from nominal flow, current, and pressure
    Kl = 0.0; % internal leakage coefficient
    %%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%% Cylinder properties %
    Apos = 0.5891 / 1550; % [m^2] area on the positive side of the cylinder
    Aneg = 0.7854 / 1550; % [m^2]
    lc = 0.1; % [m] length of the cylinder
    Vhose = 0; % [m^3] hose pipeline volume
    Fv = 1000; % [Ns/m] viscous damping coefficient, very large to make simulation reasonable   
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
      % second-order spool dynamics: K*u = 1/omega^2*xdd + 2*xi/omega*xd + x
      % Jelali and Kroll (2003) eq 4.9
      f(1) = x(2);
      f(2) = obj.omega^2*(obj.Kspool*u - 2*obj.xi/obj.omega * x(2) - x(1));

      % compute flow 
      % Jelali and Kroll (2003) eq 4.2-4.3
      if x(1) > 0
        qpos = obj.Kv*x(1)*sign(obj.p_supply-x(5))*sqrt(abs(obj.p_supply-x(5)));
        qneg = -obj.Kv*x(1)*sign(x(6)-obj.p_return)*sqrt(abs(x(6)-obj.p_return));
      else
        qpos = obj.Kv*x(1)*sign(x(5)-obj.p_return)*sqrt(abs(x(5)-obj.p_return));
        qneg = -obj.Kv*x(1)*sign(obj.p_supply-x(6))*sqrt(abs(obj.p_supply-x(6)));
      end
      
      % flow from internal leakage
      q_leakage = obj.Kl * (x(6)-x(5));
      
      % compute chamber volumes
      Vpos = obj.Vhose + obj.Apos*x(3);
      Vneg = obj.Vhose + obj.Aneg*(obj.lc-x(3));
    
      % piston motion
      % Jelali and Kroll (2003) eq 4.56
      Fext = 0;
      f(3) = x(4);
      f(4) = (obj.Apos*x(5)*1e6 - obj.Aneg*x(6)*1e6 - obj.Fv*x(4) - Fext) / obj.mass;  

      % chamber pressure dynamics
      % Jelali and Kroll (2003) eq 4.51-4.52
      f(5) = obj.beta / Vpos * (qpos - obj.Apos*x(4) + q_leakage);
      f(6) = obj.beta / Vneg * (qneg + obj.Aneg*x(4) - q_leakage);

      t
      %assert(x(3)>=0 && x(3)<=obj.lc);
    end
    
    function y=output(obj,t,x,u)
      % outputs actuator force
      f_out = obj.Apos*x(5)*1e6 - obj.Aneg*x(6)*1e6 - obj.Fv*x(4);
      y=[x;f_out];
    end
    
    function x = getInitialState(obj)
      p_neg = 0.5*obj.p_return;
      p_pos = obj.Aneg*p_neg/obj.Apos;
      x = [0;0;obj.lc/2;0;p_pos;p_neg];
    end
  end
  
end
