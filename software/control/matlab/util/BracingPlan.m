classdef BracingPlan < QPControllerPlan
  properties
    qp_input;
    qtraj;
    robot;
    zmp_data;
  end

  methods
    function obj = BracingPlan(r)
      obj.robot = r;
      d = load(obj.robot.bracing_config_file);
      obj.qtraj = d.xstar(1:obj.robot.getNumPositions());
      obj.qp_input = atlasControllers.QPInputConstantHeight();
      obj.qp_input.whole_body_data.q_des = obj.qtraj;
      obj.zmp_data = struct('A',  [zeros(2),zeros(2); zeros(4)],... % COM state map 
                        'B', [zeros(2); zeros(2)],... % COM input map
                        'C', [zeros(2),zeros(2)],... % ZMP state-output map
                        'D', zeros(2),... % ZMP input-output map
                        'x0', zeros(4,1),... % nominal state
                        'y0', zeros(2,1),... % nominal output
                        'u0', zeros(2,1),... % nominal input
                        'R', zeros(2),... % input LQR cost
                        'Qy', zeros(2),... % output LQR cost
                        'S', zeros(4),... % cost-to-go terms: x'Sx + x's1 + s2
                        's1', zeros(4,1),...
                        's1dot', zeros(4,1),... 
                        's2', 0,... 
                        's2dot', 0);
      obj.duration = inf;
      gain_set = 'bracing';
    end

    function qp_input = getQPControllerInput(obj, varargin)
      qp_input = obj.qp_input;
    end
  end
end
