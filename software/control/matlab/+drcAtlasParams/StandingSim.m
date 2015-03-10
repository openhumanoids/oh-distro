classdef StandingSim < atlasParams.Standing
  methods
    function obj = StandingSim(r)
      typecheck(r, 'DRCAtlas');
      obj = obj@atlasParams.Standing(r);

      force_controlled_joint_names = {'leg', 'arm', 'back', 'neck'};
      obj.hardware_gains = drcAtlasParams.getHardwareGains(r, force_controlled_joint_names);

      obj.body_motion(r.findLinkId('pelvis')).Kp = [150; 150; 150; 200; 200; 200];
      obj.body_motion(r.findLinkId('pelvis')).damping_ratio = 0.6;
      obj.whole_body.Kp = 150 * ones(r.getNumPositions(), 1);
      obj.whole_body.damping_ratio = 0.6;

      obj.contact_threshold = 0.002;

      obj = obj.updateKd();
    end
  end
end
