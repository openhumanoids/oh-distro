classdef SandiaHandsListener
  properties
    lc
    aggregator
    left_hand_frame
    right_hand_frame
    right_hand_map % right_hand_map(i) is the index of right_hand_frame{i} in the EST_ROBOT_STATE
    left_hand_map % left_hand_map{i} is the index of left_hand_frame{i} in the EST_ROBOT_STATE
    hand_dim
  end
  methods
    function obj = SandiaHandsListener(channel)
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.aggregator = lcm.lcm.MessageAggregator();
      obj.lc.subscribe(channel,obj.aggregator);
      obj.hand_dim = 12;
      left_hand_frame_str = cell(obj.hand_dim*2,1);
      right_hand_frame_str = cell(obj.hand_dim*2,1);
      for i = 0:3
        for j = 0:2
          ind = i*3+j+1;
          left_hand_frame_str{ind} = sprintf('left_f%d_j%d',i,j);
          left_hand_frame_str{ind+obj.hand_dim} = sprintf('left_f%d_j%ddot',i,j);
          right_hand_frame_str{ind} = sprintf('right_f%d_j%d',i,j);
          right_hand_frame_str{ind+obj.hand_dim} = sprintf('right_f%d_j%ddot',i,j);
        end
      end
      obj.left_hand_frame = CoordinateFrame('sandiaLeftHand',obj.hand_dim*2,[],left_hand_frame_str);
      obj.right_hand_frame = CoordinateFrame('sandiaRightHand',obj.hand_dim*2,[],right_hand_frame_str);
      obj.left_hand_map = containers.Map();
      obj.right_hand_map = containers.Map();
    end
    
    function data = getNextMessage(obj,t_ms)
      msg = obj.aggregator.getNextMessage(t_ms);
      if(isempty(msg))
        data = [];
      else
        data = obj.decode(drc.robot_state_t(msg.data));
      end
    end
    
    function data = decode(obj,msg)
      if(obj.left_hand_map.length()==0 || obj.right_hand_map.length() == 0)
        joint_names = cell(msg.num_joints,1);
        for i = 1:msg.num_joints
          joint_names{i} = char(msg.joint_name(i));
        end
        for i = 1:obj.hand_dim
          obj.left_hand_map(obj.left_hand_frame.coordinates{i}) = find(strcmp(joint_names,obj.left_hand_frame.coordinates{i}));
        end
        for i = 1:obj.hand_dim
          obj.right_hand_map(obj.right_hand_frame.coordinates{i}) = find(strcmp(joint_names,obj.right_hand_frame.coordinates{i}));
        end
      end
      left_hand_val = zeros(obj.hand_dim*2,1);
      right_hand_val = zeros(obj.hand_dim*2,1);
      for i = 1:obj.hand_dim
        left_hand_val(i) = msg.joint_position(obj.left_hand_map(obj.left_hand_frame.coordinates{i}));
        right_hand_val(i) = msg.joint_position(obj.right_hand_map(obj.right_hand_frame.coordinates{i}));
        left_hand_val(i+obj.hand_dim) = msg.joint_velocity(obj.left_hand_map(obj.left_hand_frame.coordinates{i}));
        right_hand_val(i+obj.hand_dim) = msg.joint_velocity(obj.right_hand_map(obj.right_hand_frame.coordinates{i}));
      end
      data.leftHand = Point(obj.left_hand_frame,left_hand_val);
      data.rightHand = Point(obj.right_hand_frame,right_hand_val);
    end
    
  end
    
end