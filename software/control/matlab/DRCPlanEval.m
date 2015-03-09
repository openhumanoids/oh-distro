classdef DRCPlanEval < atlasControllers.AtlasPlanEval
  properties
    mode = 'sim';
  end

  methods
    function obj = DRCPlanEval(r, mode, varargin)
      obj = obj@atlasControllers.AtlasPlanEval(r, varargin{:});
      assert(strcmp(obj.mode, 'sim') || strcmp(obj.mode, 'hardware'), 'bad mode: %s', mode);
      obj.mode = mode;
    end

    function qp_input = getQPControllerInput(obj, t, x)
      qp_input = getQPControllerInput@atlasControllers.AtlasPlanEval(obj, t, x);
      qp_input.param_set_name = [qp_input.param_set_name, '_', obj.mode];
    end
  end
end

