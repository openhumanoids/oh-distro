classdef DRCPlanEvalData < ControllerData
% Container class for the PlanEval's queue of plans. This class is necessary
% in order to make the plan queue a handle object, in order to make it
% persistent across multiple calls to PlanEval.getCurrentPlan
  properties
    plan_queue
    pause_state
    qp_input
    t
    x
    contact_force_detected
    last_status_msg_time
  end

  properties(Constant)
   PAUSE_NONE = 0;
   PAUSE_NOW = 1;
   STOP_WALKING_ASAP = 2;
 end


  methods
    function obj = DRCPlanEvalData(data)
      data = applyDefaults(data, struct('plan_queue', {{}},...
                                      'qp_input', {[]},...
                                      'pause_state', {DRCPlanEvalData.PAUSE_NONE}));
      obj = obj@ControllerData(data);
    end

    function verifyControllerData(obj, data)
      for j = 1:length(data.plan_queue)
        typecheck(data.plan_queue{j}, 'QPControllerPlan');
      end
    end
  end
end
