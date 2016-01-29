classdef ValkyrieState < LCMCoordinateFrame & Singleton
  methods
    function obj = ValkyrieState(r)
      typecheck(r,'OHValkyrie');

      obj = obj@LCMCoordinateFrame('drcFrames.ValkyrieState',r.getNumStates(),'x');
      obj = obj@Singleton();

      joint_names = {r.getStateFrame.getCoordinateNames{1:getNumPositions(r)}}';
      if isempty(obj.lcmcoder)  % otherwise I had a singleton

        coder = drc.control.RobotStateCoder(joint_names);

        obj.setLCMCoder(JLCMCoder(coder));
%        obj.setCoordinateNames(r.getStateFrame.getCoordinateNames);
        obj.setCoordinateNames({r.getStateFrame.getCoordinateNames{1:(r.getNumPositions() + r.getNumVelocities())}}');
        
        obj.setDefaultChannel('EST_ROBOT_STATE');
      end
    end
  end
end

