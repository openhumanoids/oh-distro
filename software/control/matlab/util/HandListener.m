classdef HandListener
  % @param left_hand_frame    -- A coordinate frame of the left hand
  % @param hand_frame   -- A coordinate frame of the right hand
  % @param hand_map      -- hand_map(i) is the index of
  %                         hand_frame{i} in the lcm message
  % @param hand_dim            -- DOF of hand knuckles
  % @param hand_mode         -- - 0 no hand
  %                                 - 1 sandia hand
  %                                 - 2 irobot hand
  properties
    lc
    aggregator
    hand_frame
    hand_map 
    hand_dim
    hand_mode
  end
  methods
    function obj = HandListener(hand_mode,prefix,channel)
      % prefix = 'left' or 'right'
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.aggregator = lcm.lcm.MessageAggregator();
      obj.lc.subscribe(channel,obj.aggregator);
      
      
      if(hand_mode == 0)
        obj.hand_dim = 0;
        hand_frame_str = {};
        display(sprintf('No hand listener for %s hand',prefix));
      elseif(hand_mode == 1)
        obj.hand_dim = 12;
        hand_frame_str = cell(obj.hand_dim*2,1);
        for i = 0:3
          for j = 0:2
            ind = i*3+j+1;
            hand_frame_str{ind} = sprintf('%s_f%d_j%d',prefix,i,j);
            hand_frame_str{ind+obj.hand_dim} = sprintf('%s_f%d_j%ddot',prefix,i,j);
          end
        end
        display(sprintf('construct sandia hand listener for %s hand',prefix));
        obj.hand_frame = CoordinateFrame(sprintf('sandia%sHand',prefix),obj.hand_dim*2,[],hand_frame_str);
      elseif(hand_mode == 2)
        obj.hand_dim = 8;
        hand_frame_str = cell(obj.hand_dim*2,1);
        for i = 0:1
          ind = i*3+1;
          hand_frame_str{ind} = sprintf('%s_finger[%d]/joint_base_rotation',prefix,i);
          hand_frame_str{ind+obj.hand_dim} = sprintf('%s_finger[%d]/joint_base_rotationdot',prefix,i);
          ind = i*3+2;
          hand_frame_str{ind} = sprintf('%s_finger[%d]/joint_base',prefix,i);
          hand_frame_str{ind+obj.hand_dim} = sprintf('%s_finger[%d]/joint_basedot',prefix,i);
          ind = i*3+3;
          hand_frame_str{ind} = sprintf('%s_finger[%d]/joint_flex',prefix,i);
          hand_frame_str{ind+obj.hand_dim} = sprintf('%s_finger[%d]/joint_flexdot',prefix,i);
        end
        hand_frame_str{7} = sprintf('%s_finger[2]/joint_base',prefix);
        hand_frame_str{7+obj.hand_dim} = sprintf('%s_finger[2]/joint_basedot',prefix);
        hand_frame_str{8} = sprintf('%s_finger[2]/joint_flex',prefix);
        hand_frame_str{8+obj.hand_dim} = sprintf('%s_finger[2]/joint_flexdot',prefix);
        display(sprintf('construct irobot hand listener for %s hand',prefix));
        obj.hand_frame = CoordinateFrame(sprintf('irobot%sHand',prefix),obj.hand_dim*2,[],hand_frame_str);
      end
      obj.hand_mode = hand_mode;
      obj.hand_map = containers.Map();
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
      if(obj.hand_map.length()==0 )
        joint_names = cell(msg.num_joints,1);
        for i = 1:msg.num_joints
          joint_names{i} = char(msg.joint_name(i));
        end
        if(obj.hand_mode ~= 0 )
          for i = 1:obj.hand_dim
            obj.hand_map(obj.hand_frame.coordinates{i}) = find(strcmp(joint_names,obj.hand_frame.coordinates{i}));
          end
        end
      end
      hand_val = zeros(obj.hand_dim*2,1);
      for i = 1:obj.hand_dim
        if(obj.hand_mode ~= 0)
          hand_val(i) = msg.joint_position(obj.hand_map(obj.hand_frame.coordinates{i}));
          hand_val(i+obj.hand_dim) = msg.joint_velocity(obj.hand_map(obj.hand_frame.coordinates{i}));
        end
      end
      data = Point(obj.hand_frame,hand_val);
    end
    
  end
    
end