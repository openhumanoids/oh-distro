classdef StateCorrupter < DrakeSystem
  
  methods
    function obj = StateCorrupter(robot)
      typecheck(robot,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
      obj = obj@DrakeSystem(0,0,getNumStates(robot),getNumStates(robot),true,true);
      obj = setInputFrame(obj,getStateFrame(robot));
      obj = setOutputFrame(obj,getStateFrame(robot));
      obj.robot = robot;
    end
    
    function y = output(obj,t,~,u)
      y = u;
    end
  end
  
  properties
    robot
  end
end