classdef WalkingSim < atlasParams.Walking
  methods
    function obj = WalkingSim(r)
      typecheck(r, 'DRCAtlas');
      obj = obj@atlasParams.Walking(r);

      force_controlled_joint_names = {'leg', 'arm', 'back', 'neck'};
      obj.hardware = drcAtlasParams.getHardwareParams(r, force_controlled_joint_names);

      obj.body_motion(r.findLinkId('pelvis')).Kp = [nan; nan; 150; 200; 200; 200];
      obj.body_motion(r.findLinkId('pelvis')).damping_ratio = 0.6;

      obj = obj.updateKd();
    end
  end
end
