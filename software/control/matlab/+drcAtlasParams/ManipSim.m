classdef ManipSim < drcAtlasParams.StandingSim
  methods
    function obj = ManipSim(r)
      obj = obj@drcAtlasParams.StandingSim(r);
      obj.body_motion(r.findLinkId('r_hand')).weight = 0.001;
      obj.body_motion(r.findLinkId('l_hand')).weight = 0.001;
    end
  end
end
