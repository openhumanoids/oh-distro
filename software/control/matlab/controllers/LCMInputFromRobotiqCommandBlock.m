classdef LCMInputFromRobotiqCommandBlock < LCMInputFromRobotiqCommandBlockBase
  
  properties
    in_indices;
    out_indices;
    kp
    kd
  end
  
  methods
    function obj = LCMInputFromRobotiqCommandBlock(r, handedness, options)
      obj = obj@LCMInputFromRobotiqCommandBlockBase(r, handedness, options);
      % Precompute indices for our position control
      obj.in_indices = [obj.getInputFrame.findCoordinateIndex('finger_1_joint_0');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_0');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_0');
                        obj.getInputFrame.findCoordinateIndex('finger_1_joint_0dot');
                        obj.getInputFrame.findCoordinateIndex('finger_2_joint_0dot');
                        obj.getInputFrame.findCoordinateIndex('finger_middle_joint_0dot')];
      obj.out_indices = [obj.getOutputFrame.findCoordinateIndex('finger_1_control_torque');
                        obj.getOutputFrame.findCoordinateIndex('finger_2_control_torque');
                        obj.getOutputFrame.findCoordinateIndex('finger_middle_control_torque')];
      obj.kp = -10;
      obj.kd = 0.5;
    end
    
    function varargout=mimoOutput(obj,t,~,hand_state)
      obj = obj.grabLCMMessage();
      pos = hand_state(obj.in_indices(1:3));
      vel = hand_state(obj.in_indices(4:6));
      efforts = zeros(obj.getNumOutputs, 1);
      % pd position control
      efforts(obj.out_indices) = obj.kp*(pos-(2.5*obj.target_positions-1.0)) - obj.kd*vel;
      varargout = {efforts};
    end
  end
  
end
