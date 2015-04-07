classdef QuasistaticSim < drcAtlasParams.StandingSim
  methods
    function obj = QuasistaticSim(r)
      obj = obj@drcAtlasParams.StandingSim(r);

      % switch from body motion weight on pelvis to whole body on the floating base
      obj.body_motion(r.findLinkId('pelvis')).weight = 0;
      obj.body_motion(r.foot_body_id.right).weight = 0.01;
      obj.body_motion(r.findLinkId('pelvis')).Kp = [150; 150; 150; 200; 200; 200];
      obj.body_motion(r.findLinkId('pelvis')).damping_ratio = 0.6;

      floating_base_idx = [1:6];
      obj.whole_body.w_qdd(floating_base_idx) = 0.01*ones(6,1);
      obj.whole_body.Kp(floating_base_idx) = [150; 150; 150; 200; 200; 200];
      obj.whole_body.damping_ratio = 0.6;
    end
  end
end
