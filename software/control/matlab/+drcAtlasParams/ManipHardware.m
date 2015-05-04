classdef ManipHardware < drcAtlasParams.StandingHardware;
  methods
    function obj = ManipHardware(r)
      obj = obj@drcAtlasParams.StandingHardware(r);
      obj.body_motion(r.findLinkId('r_hand')).weight = 0.001;
      obj.body_motion(r.findLinkId('l_hand')).weight = 0.001;
    end
  end
end
