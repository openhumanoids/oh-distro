function frame = handFrame(hand_mode,prefix)
if(hand_mode == 0)
  hand_dim = 0;
  hand_frame_str = {};
  frame = CoordinateFrame(sprintf('no%sHand',prefix),hand_dim*2,[],hand_frame_str);
elseif(hand_mode == 1)
  hand_dim = 12;
  hand_frame_str = cell(hand_dim*2,1);
  for i = 0:3
    for j = 0:2
      ind = i*3+j+1;
      hand_frame_str{ind} = sprintf('%s_f%d_j%d',prefix,i,j);
      hand_frame_str{ind+hand_dim} = sprintf('%s_f%d_j%ddot',prefix,i,j);
    end
  end
  frame = CoordinateFrame(sprintf('sandia%sHand',prefix),hand_dim*2,[],hand_frame_str);
elseif(hand_mode == 2)
%   hand_dim = 8;
%     hand_frame_str = cell(hand_dim*2,1);
%     for i = 0:1
%       ind = i*3+1;
%       hand_frame_str{ind} = sprintf('%s_finger[%d]/joint_base_rotation',prefix,i);
%       hand_frame_str{ind+obj.hand_dim} = sprintf('%s_finger[%d]/joint_base_rotationdot',prefix,i);
%       ind = i*3+2;
%       hand_frame_str{ind} = sprintf('%s_finger[%d]/joint_base',prefix,i);
%       hand_frame_str{ind+obj.hand_dim} = sprintf('%s_finger[%d]/joint_basedot',prefix,i);
%       ind = i*3+3;
%       hand_frame_str{ind} = sprintf('%s_finger[%d]/joint_flex',prefix,i);
%       hand_frame_str{ind+obj.hand_dim} = sprintf('%s_finger[%d]/joint_flexdot',prefix,i);
%     end
%     hand_frame_str{7} = sprintf('%s_finger[2]/joint_base',prefix);
%     hand_frame_str{7+hand_dim} = sprintf('%s_finger[2]/joint_basedot',prefix);
%     hand_frame_str{8} = sprintf('%s_finger[2]/joint_flex',prefix);
%     hand_frame_str{8+hand_dim} = sprintf('%s_finger[2]/joint_flexdot',prefix);
  hand_dim = 0;
  hand_frame_str = {};
  frame = CoordinateFrame(sprintf('irobot%sHand',prefix),hand_dim*2,[],hand_frame_str);
elseif(hand_mode == 3)
  hand_dim = 0;
  hand_frame_str = {};
  frame = CoordinateFrame(sprintf('irobotHose%sHand',prefix),hand_dim*2,[],hand_frame_str);
elseif(hand_mode == 4)
  hand_dim = 0;
  hand_frame_str = {};
  frame = CoordinateFrame(sprintf('robotiq%sHand',prefix),hand_dim*2,[],hand_frame_str);  
end
end
