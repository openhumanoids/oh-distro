classdef DummySys < DrakeSystem

  properties
    robot;
  end

  methods
    function obj = DummySys(r,options)
      typecheck(r, 'Atlas');

      input_frame = r.getStateFrame();
      ut = UtimeFrame();
      obj = obj@DrakeSystem(0,0,input_frame.dim,1,true,true);
      obj = setInputFrame(obj, input_frame);
      obj = setOutputFrame(obj, ut);

      if isfield(options, 'dt')
        typecheck(options.dt, 'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.005;
      end

      obj.robot = r;
      obj = setSampleTime(obj,[dt;0]);
    end

    function y = output(obj, t, ~, ~)
      y = t*1e6;
    end
  end
end
