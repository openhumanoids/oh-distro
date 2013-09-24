classdef EEWrenchListener
  properties
    lc
    aggregator
  end
  
  methods
    function obj = EEWrenchListener(channel_name)
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.aggregator = lcm.lcm.MessageAggregator();
      obj.lc.subscribe(channel_name,obj.aggregator);
    end
    
    function x = getNextMessage(obj,t_ms)
      msg_raw = obj.aggregator.getNextMessage(t_ms);
      if(isempty(msg_raw))
        x = [];
      else
        msg = drc.robot_state_t(msg_raw.data);
        x = EEWrenchListener.decode(msg);
      end
    end
  end
  methods(Static)    
    function x = decode(msg)
      lh_wrench = zeros(6,1);
      lh_wrench(1) = msg.force_torque.l_hand_force(1);
      lh_wrench(2) = msg.force_torque.l_hand_force(2);
      lh_wrench(3) = msg.force_torque.l_hand_force(3);
      lh_wrench(4) = msg.force_torque.l_hand_torque(1);
      lh_wrench(5) = msg.force_torque.l_hand_torque(2);
      lh_wrench(6) = msg.force_torque.l_hand_torque(3);
      
      rh_wrench = zeros(6,1);
      rh_wrench(1) = msg.force_torque.r_hand_force(1);
      rh_wrench(2) = msg.force_torque.r_hand_force(2);
      rh_wrench(3) = msg.force_torque.r_hand_force(3);
      rh_wrench(4) = msg.force_torque.r_hand_torque(1);
      rh_wrench(5) = msg.force_torque.r_hand_torque(2);
      rh_wrench(6) = msg.force_torque.r_hand_torque(3);      
     
      lf_wrench = zeros(6,1);
      lf_wrench(3) = msg.force_torque.l_foot_force_z;
      lf_wrench(4) = msg.force_torque.l_foot_torque_x;
      lf_wrench(5) = msg.force_torque.l_foot_torque_y;
      
      rf_wrench = zeros(6,1);
      rf_wrench(3) = msg.force_torque.r_foot_force_z;
      rf_wrench(4) = msg.force_torque.r_foot_torque_x;
      rf_wrench(5) = msg.force_torque.r_foot_torque_y; 
      
      x.lh = lh_wrench;
      x.rh = rh_wrench;
      x.lf = lf_wrench;
      x.rf = rf_wrench;
    end
  end
end
