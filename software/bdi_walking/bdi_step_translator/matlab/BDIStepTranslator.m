classdef BDIStepTranslator < DRCPlanner
  properties
    lc
  end

  methods 
    function obj = BDIStepTranslator()
      obj = obj@DRCPlanner();
      obj = addInput(obj, 'foot_plan', 'APPROVED_FOOTSTEP_PLAN', 'drc.footstep_plan_t', 1, 1, 1);
      obj.lc = lcm.lcm.LCM.getSingleton();
    end

    function plan(obj, data)
      NUM_REQUIRED_WALK_STEPS = 4;
      footsteps = FootstepPlanListener.decodeFootstepPlan(data.foot_plan);
      try
        sizecheck(footsteps, NUM_REQUIRED_WALK_STEPS+2);
      catch
        msg ='ERROR: Footstep plan must be exactly 4 steps for BDI translation'; disp(msg); send_status(6,0,0,msg);
        return;
      end

      walk_param_msg = BDIStepTranslator.translateFootsteps(footsteps);
      obj.lc.publish('ATLAS_WALK_PARAMS', walk_param_msg);
    end
  end

  methods (Static)
    function walk_param_msg = translateFootsteps(footsteps)
      NUM_REQUIRED_WALK_STEPS = 4;
      sizecheck(footsteps, NUM_REQUIRED_WALK_STEPS+2);
      walk_param_msg = drc.atlas_behavior_walk_params_t();
      walk_param_msg.num_required_walk_steps = NUM_REQUIRED_WALK_STEPS;
      walk_param_msg.step_queue = javaArray('drc.atlas_step_data_t', NUM_REQUIRED_WALK_STEPS);
      walk_param_msg.use_relative_step_height = 0;
      walk_param_msg.use_demo_walk = 0;
      for j = 1:NUM_REQUIRED_WALK_STEPS
        step_data = drc.atlas_step_data_t();
        step_data.step_index = j;
        step_data.foot_index = footsteps(j+2).is_right_foot;
        %% TODO: computer step duration based on target foot speed
        step_data.duration = 0.7;
        step_data.position = footsteps(j+2).pos(1:3);
        step_data.yaw = footsteps(j+2).pos(6);
        step_data.normal = rpy2rotmat(footsteps(j+2).pos(4:6)) * [0;0;1];
        step_data.swing_height = 0.05; % currently using BDI default % footsteps(j+2).step_height;
        walk_param_msg.step_queue(j) = step_data;
      end
    end
  end
end
