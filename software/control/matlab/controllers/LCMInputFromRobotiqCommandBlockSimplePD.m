classdef LCMInputFromRobotiqCommandBlockSimplePD < LCMInputFromRobotiqCommandBlockBase
  
  properties
    in_indices;
    out_indices;
    pp; %polynomial spline for generating q_desired
    kp;
    kd;
  end
  
  methods
    function obj = LCMInputFromRobotiqCommandBlockSimplePD(r, options)
      obj = obj@LCMInputFromRobotiqCommandBlockBase(r, options);
      q_closed = [1.2427, 1.0864, -0.1336, 1.2427, 1.0864,-0.1336, 1.2427, 0.9114, 0.0000]';
      q_open = zeros(numel(q_closed), 1);
      obj.pp = spline([0, 1], [q_open, q_closed]);

      obj.in_indices = [obj.getInputFrame.findCoordinateIndex('finger_middle_joint_0');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_1');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_2');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_0');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_1');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_2');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_0');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_1');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_2');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_0dot');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_1dot');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_2dot');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_0dot');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_1dot');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_2dot');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_0dot');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_1dot');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_2dot')];
      obj.out_indices = [obj.getOutputFrame.findCoordinateIndex('finger_middle_joint_0');
                        obj.getOutputFrame.findCoordinateIndex('finger_middle_joint_1');
                        obj.getOutputFrame.findCoordinateIndex('finger_middle_joint_2');
                        obj.getOutputFrame.findCoordinateIndex('finger_1_joint_0');
                        obj.getOutputFrame.findCoordinateIndex('finger_1_joint_1');
                        obj.getOutputFrame.findCoordinateIndex('finger_1_joint_2');
                        obj.getOutputFrame.findCoordinateIndex('finger_2_joint_0');
                        obj.getOutputFrame.findCoordinateIndex('finger_2_joint_1');
                        obj.getOutputFrame.findCoordinateIndex('finger_2_joint_2')];
      obj.kp = -3;
      obj.kd = 0.1;
    end
    
    function varargout=mimoOutput(obj,t,~,hand_state)
      obj = obj.grabLCMMessage();
      pos = hand_state(obj.in_indices(1:9));
      vel = hand_state(obj.in_indices(10:18));
      q_desired = ppval(obj.pp, obj.target_positions(1));
      efforts = zeros(obj.getNumOutputs, 1);
      efforts(obj.out_indices) = obj.kp*(pos-q_desired(obj.in_indices(1:9))) - obj.kd*vel;
      varargout = {efforts};
    end
  end
  
end
