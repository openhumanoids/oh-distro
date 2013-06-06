classdef AtlasHandForceTorque < LCMCoordinateFrameWCoder & Singleton
  
  methods
    function obj=AtlasHandForceTorque()
      contacts = {'l_hand','r_hand'};
      coder = ContactStateCoder('atlas', contacts);
      
      coords = {'fx','fy','fz','tx','ty','tz'};
      coordinates={};
      for i=1:length(contacts)
        coordinates = horzcat(coordinates,cellfun(@(a) [contacts{i},'_',a],coords,'UniformOutput',false));
      end
      obj = obj@LCMCoordinateFrameWCoder('AtlasHandForceTorque',length(coordinates),'f',JLCMCoder(coder));
      obj.setCoordinateNames(coordinates);
      obj.setDefaultChannel('EST_ROBOT_STATE');
    end
  end
end
