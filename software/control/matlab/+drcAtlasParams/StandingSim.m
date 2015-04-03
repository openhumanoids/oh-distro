classdef StandingSim < atlasParams.Standing
  methods
    function obj = StandingSim(r)
      typecheck(r, 'DRCAtlas');
      obj = obj@atlasParams.Standing(r);

      force_controlled_joint_names = {'leg', 'arm', 'back', 'neck'};
      obj.hardware = drcAtlasParams.getHardwareParams(r, force_controlled_joint_names);

      obj = obj.updateKd();
    end
  end
end
