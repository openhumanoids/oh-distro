classdef SimplePDController < DrakeSystem
  % outputs a desired q_ddot (including floating dofs)
  properties
    nq;
    Kp;
    Kd;
    controller_data; % pointer to shared data handle containing qtraj, ti_flag
  end
  
  methods
    function obj = SimplePDController(r,controller_data)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');

      input_frame = r.getStateFrame;
      coords = AtlasCoordinates(r);
      obj = obj@DrakeSystem(0,0,input_frame.dim,coords.dim,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,coords);

      obj.controller_data = controller_data;
      obj.nq = getNumDOF(r);
      obj.Kp = 200*eye(obj.nq);
      obj.Kd = 20.0*eye(obj.nq);
      obj.Kp(1:2,1:2) = zeros(2); % ignore x,y
      obj.Kd(1:2,1:2) = zeros(2); % ignore x,y
    end
   
  	function y=output(obj,t,~,x)
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);

      cdata = obj.controller_data.getData();

      if cdata.ti_flag
        q_des = cdata.qtraj;
      else
        q_des = cdata.qtraj.eval(t);
      end
      
      err_q = q_des - q;
      y = obj.Kp*err_q - obj.Kd*qd;
    end
  end
  
end
