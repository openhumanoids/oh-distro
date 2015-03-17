classdef FrozenPlan < QPControllerPlan
  properties
    frozen_qp_input;
  end

  methods
    function obj = FrozenPlan(qp_input)
      for j = 1:length(qp_input.body_motion_data)
        qp_input.body_motion_data(j).coefs(:,:,1:end-1) = 0;
      end
      obj.frozen_qp_input = qp_input;
      obj.duration = inf;
    end

    function qp_input = getQPControllerInput(obj, varargin)
      qp_input = obj.frozen_qp_input;
    end
  end
end

