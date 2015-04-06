classdef RecoverySim < atlasParams.Recovery
  methods
    function obj = RecoverySim(r)
      typecheck(r, 'DRCAtlas');
      obj = obj@atlasParams.Recovery(r);

      force_controlled_joint_names = {'leg', 'arm', 'back', 'neck'};
      obj.hardware = drcAtlasParams.getHardwareParams(r, force_controlled_joint_names);
      obj.whole_body.w_qdd(r.findPositionIndices('back_bkx')) = 0.001; % use back x for stabilization
      %obj.body_motion(r.findLinkId('pelvis')).weight = 0.075;
      obj = obj.updateKd();
    end
  end
end