classdef SandiaHandsListener
  % @param left_hand_frame    -- A coordinate frame of the left hand
  % @param right_hand_frame   -- A coordinate frame of the right hand
  % @param left_hand_map      -- left_hand_map(i) is the index of
  %                              left_hand_frame{i} in the lcm message
  % @param right_hand_map      -- right_hand_map(i) is the index of
  %                              right_hand_frame{i} in the lcm message
  % @param hand_dim            -- DOF of hand knuckles
  % @param hand                -- - 1 left hand only
  %                               - 2 right hand only
  %                               - 3 both hands
  properties
    lc
    aggregator
    left_hand_frame
    right_hand_frame
    right_hand_map % right_hand_map(i) is the index of right_hand_frame{i} in the EST_ROBOT_STATE
    left_hand_map % left_hand_map{i} is the index of left_hand_frame{i} in the EST_ROBOT_STATE
    hand_dim
    hand
  end
  methods
    function obj = SandiaHandsListener(hand,channel)
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
      if(strcmp(hand,'left'))
        obj.left_hand_frame = CoordinateFrame('sandiaLeftHand',obj.hand_dim*2,[],left_hand_frame_str);
        obj.right_hand_frame = CoordinateFrame('sandiaRightHand',0,[],[]);
        obj.hand = 1;
      elseif(strcmp(hand,'right'))
        obj.right_hand_frame = CoordinateFrame('sandiaRightHand',obj.hand_dim*2,[],right_hand_frame_str);
        obj.left_hand_frame = CoordinateFrame('sandiaLeftHand',0,[],[]);
        obj.hand = 2;
      elseif(strcmp(hand,'both'))
        obj.left_hand_frame = CoordinateFrame('sandiaLeftHand',obj.hand_dim*2,[],left_hand_frame_str);
        obj.right_hand_frame = CoordinateFrame('sandiaRightHand',obj.hand_dim*2,[],right_hand_frame_str);
        obj.hand = 3;
      else
        error('hand should be either ''left'', ''right'' or ''both''');
      end
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
      if(obj.left_hand_map.length()==0 && obj.right_hand_map.length() == 0)
        joint_names = cell(msg.num_joints,1);
        for i = 1:msg.num_joints
          joint_names{i} = char(msg.joint_name(i));
        end
        if(obj.hand == 1 || obj.hand == 3)
          for i = 1:obj.hand_dim
            obj.left_hand_map(obj.left_hand_frame.coordinates{i}) = find(strcmp(joint_names,obj.left_hand_frame.coordinates{i}));
          end
        end
        if(obj.hand == 2 || obj.hand == 3)
          for i = 1:obj.hand_dim
            obj.right_hand_map(obj.right_hand_frame.coordinates{i}) = find(strcmp(joint_names,obj.right_hand_frame.coordinates{i}));
          end
        end
      end
      if(obj.hand == 1 || obj.hand == 3)
        left_hand_val = zeros(obj.hand_dim*2,1);
      else
        left_hand_val = [];
      end
      if(obj.hand == 2 || obj.hand == 3)
        right_hand_val = zeros(obj.hand_dim*2,1);
      else
        right_hand_val = [];
      end
      for i = 1:obj.hand_dim
        if(obj.hand == 1 || obj.hand == 3)
          left_hand_val(i) = msg.joint_position(obj.left_hand_map(obj.left_hand_frame.coordinates{i}));
          left_hand_val(i+obj.hand_dim) = msg.joint_velocity(obj.left_hand_map(obj.left_hand_frame.coordinates{i}));
        end
        if(obj.hand == 2 || obj.hand == 3)
          right_hand_val(i) = msg.joint_position(obj.right_hand_map(obj.right_hand_frame.coordinates{i}));
          right_hand_val(i+obj.hand_dim) = msg.joint_velocity(obj.right_hand_map(obj.right_hand_frame.coordinates{i}));
        end
      end
      if(obj.hand == 1 || obj.hand == 3)
        data.leftHand = Point(obj.left_hand_frame,left_hand_val);
      else
        data.leftHand = [];
      end
      if(obj.hand == 2 || obj.hand == 3)
        data.rightHand = Point(obj.right_hand_frame,right_hand_val);
      else
        data.rightHand = [];
      end
    end
    
  end
    
end