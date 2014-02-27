classdef AtlasForceTorque < LCMCoordinateFrame & Singleton
  
  methods
    function obj=AtlasForceTorque()

      coordinates = {'l_foot_fz','l_foot_tx','l_foot_ty',...
                'r_foot_fz','r_foot_tx','r_foot_ty',...
                'l_hand_fx','l_hand_fy','l_hand_fz',...
                'l_hand_tx','l_hand_ty','l_hand_tz',...
                'r_hand_fx','r_hand_fy','r_hand_fz',...
                'r_hand_tx','r_hand_ty','r_hand_tz'};
      
      obj = obj@LCMCoordinateFrame('AtlasForceTorque',length(coordinates),'f');
      obj = obj@Singleton();
      
      if isempty(obj.lcmcoder)
        coder = drc.control.ForceTorqueStateCoder();
        setLCMCoder(obj,JLCMCoder(coder));
        obj.setCoordinateNames(coordinates);
        obj.setDefaultChannel('EST_ROBOT_STATE');
      end
    end
  end
end
