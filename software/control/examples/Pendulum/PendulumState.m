classdef PendulumState < LCMCoordinateFrameWCoder & Singleton
  
  methods
    function obj=PendulumState()
      coder = RobotStateCoder('Pendulum',{'theta'});
      obj = obj@LCMCoordinateFrameWCoder('PendulumState',2,'x',JLCMCoder(coder));
      obj.setAngleFlags([true;false]);
    end
    
    function str = defaultChannel(obj)
      str = 'EST_ROBOT_STATE';
    end
  end
end
