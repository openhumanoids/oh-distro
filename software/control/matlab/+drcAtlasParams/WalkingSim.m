classdef WalkingSim < atlasParams.Walking
  methods
    function obj = WalkingSim(r)
      typecheck(r, 'DRCAtlas');
      obj = obj@atlasParams.Walking(r);

      force_controlled_joint_names = {'leg', 'arm', 'back', 'neck'};
      obj.hardware = drcAtlasParams.getHardwareParams(r, force_controlled_joint_names);

      obj = obj.updateKd();
    end
  end
end
