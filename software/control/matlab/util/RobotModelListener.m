classdef RobotModelListener
  properties
    lc
    aggregator
  end
  
  methods
    function obj = RobotModelListener(channel)
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.aggregator = lcm.lcm.MessageAggregator();
      obj.lc.subscribe(channel,obj.aggregator);
    end
    
    function data = getNextMessage(obj,t_ms)
      msg = obj.aggregator.getNextMessage(t_ms);
      if(isempty(msg))
        data = [];
      else
        data = RobotModelListener.decode(drc.robot_urdf_t(msg.data));
      end
    end
  end
  
  methods(Static)
    function data = decode(msg)
      
      if(msg.left_hand == msg.LEFT_NONE)
        data.left_hand_mode = 0;
      elseif(msg.left_hand == msg.LEFT_SANDIA)
        data.left_hand_mode = 1;
      elseif(msg.left_hand == msg.LEFT_IROBOT)
        data.left_hand_mode = 2;
      elseif(msg.left_hand == msg.LEFT_ROBOTIQ)
        data.left_hand_mode = 4;  
      end
      if(msg.right_hand == msg.RIGHT_NONE)
        data.right_hand_mode = 0;
      elseif(msg.right_hand == msg.RIGHT_SANDIA)
        data.right_hand_mode = 1;
      elseif(msg.right_hand == msg.RIGHT_IROBOT)
        data.right_hand_mode = 2;
      elseif(msg.right_hand == msg.RIGHT_ROBOTIQ)
        data.right_hand_mode = 4;   
      end
      data.robot_name = char(msg.robot_name);
    end
  end
end
