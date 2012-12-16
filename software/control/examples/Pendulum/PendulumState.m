classdef PendulumState < RobotStateFrame
  
  methods
    function obj=PendulumState()
      obj = obj@RobotStateFrame('Pendulum',{'theta'});
      obj.setAngleFlags([true;false]);
    end
  end
end
