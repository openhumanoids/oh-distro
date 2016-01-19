classdef DRCInstantaneousQPController < bipedControllers.InstantaneousQPController
  % A standalone InstantaneousQPController (DRC-specific) extension 
  % that knows how to read plans from LCM, and offers
  % a blocking run() method to receive and publish
  % plans in a tight loop.

  properties  
    lc
    
    qp_controller_input_monitor;
    qp_controller_input_channel;

    atlas_state_monitor;
    atlas_state_channel;

    cmd_pub_coder;
    command_channel;
  end

  methods
    function obj = DRCInstantaneousQPController(r)
      typecheck(r,'DRCAtlas');
      obj = obj@bipedControllers.InstantaneousQPController(r);
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.qp_controller_input_monitor = drake.util.MessageMonitor(drake.lcmt_qp_controller_input, 'timestamp');
      obj.qp_controller_input_channel = 'QP_CONTROLLER_INPUT';

      obj.atlas_state_monitor = obj.robot.getStateFrame;
      obj.atlas_state_channel = 'EST_ROBOT_STATE';

      % Set up a command coder
      obj.cmd_pub_coder = drcFrames.AtlasInput(r);
      obj.command_channel = 'ATLAS_COMMAND';
    end

    function run(obj)
      % subscribe to robot state + qp controller input
      obj.atlas_state_monitor.subscribe(obj.atlas_state_channel);
      obj.lc.subscribe(obj.qp_controller_input_channel, obj.qp_controller_input_monitor);
      
      S = load(obj.robot.fixed_point_file);
      x0 = double(S.xstar);
      stand_plan = StandingPlan.from_standing_state(x0, obj.robot);
      rpc = robot.getRobotPropertyCache();
      
      disp('DRC Inst QP Controller now listening');

      qp_input = [];
      x = [];
      t = [];
      
      while 1
%         % check for plan
        qp_input_new = obj.qp_controller_input_monitor.getMessage();
        if ~isempty(qp_input_new)
          qp_input = drake.lcmt_qp_controller_input(qp_input_new);
          qp_input = bipedControllers.QPInputConstantHeight.from_lcm(qp_input);
        end
        if (isempty(qp_input))
          continue;
        end
        
        % check for new state
        [new_x, new_t] = obj.atlas_state_monitor.getNextMessage(5);
        if (~isempty(new_x))
          x = new_x; t = new_t;
        else
          continue;
        end
        if isempty(x)
          continue
        end
        
        % this part makes the robot not fall over
        [y, v_ref] = obj.updateAndOutput(t, x, qp_input, [-1;-1]);

        obj.cmd_pub_coder.publish(t, y, obj.command_channel);
        
        %disp('\\O\\ /O/')
      end
    end
  end
  
end
