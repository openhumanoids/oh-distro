classdef QuasistaticHardware < atlasParams.Base
  methods
    function obj = QuasistaticHardware(r)
      typecheck(r, 'DRCAtlas');
      obj = obj@atlasParams.Base(r);

      force_controlled_joint_names = {'leg', 'back_bkx'};
      obj.hardware = drcAtlasParams.getHardwareParams(r, force_controlled_joint_names);

      obj.whole_body.Kp = zeros(r.getNumPositions(), 1);
      obj.whole_body.Kp(r.findPositionIndices('back_bkx')) = 50;
      obj.whole_body.damping_ratio = 0.5;

      % set the body motion weight on the pelvis to zero, 
      % and the whole body weight on floating_base to be what they were for pelvis???
      obj.body_motion(r.findLinkId('pelvis')).weight = 0.0;
      % obj.body_motion(r.foot_body_id.right).weight = 0.01;
      % obj.body_motion(r.findLinkId('pelvis')).Kp = [150; 150; 150; 200; 200; 200];
      % obj.body_motion(r.findLinkId('pelvis')).damping_ratio = 0.6;
      floating_base_idx = [1:6];
      obj.whole_body.w_qdd(floating_base_idx) = 0.01*ones(6,1);
      obj.whole_body.Kp(floating_base_idx) = 40*ones(6,1);
      obj.whole_body.damping_ratio = 0.6;

      % integral gains for position controlled joints
      integral_gains = zeros(getNumPositions(r),1);
      integral_clamps = zeros(getNumPositions(r),1);
      arm_ind = findPositionIndices(r,'arm');
      back_ind = findPositionIndices(r,'back');
      integral_gains(arm_ind) = 1.75; % TODO: generalize this
      integral_gains(back_ind) = 0.3;
      integral_clamps(arm_ind) = 0.2;
      integral_clamps(back_ind) = 0.2;
      obj.whole_body.integrator.gains = integral_gains;
      obj.whole_body.integrator.clamps = integral_clamps;

      obj = obj.updateKd();

    end
  end
end

