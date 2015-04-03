classdef WalkingHardware < atlasParams.Base
  methods
    function obj = WalkingHardware(r)
      typecheck(r, 'DRCAtlas');
      obj = obj@atlasParams.Base(r);

      force_controlled_joint_names = {'leg', 'back_bkx'};
      obj.hardware = drcAtlasParams.getHardwareParams(r, force_controlled_joint_names);

      obj.contact_threshold = 0.001;

      obj.whole_body.Kp = zeros(r.getNumPositions(), 1);
      obj.whole_body.Kp(r.findPositionIndices('back_bkx')) = 50;
      obj.whole_body.damping_ratio = 0.5;
      obj.whole_body.w_qdd = zeros(r.getNumVelocities(), 1);
      if (r.getNumVelocities() ~= r.getNumPositions())
        error('this code calls findPositionIndices, which is no longer equivalent to findVelocityIndices');
      end
      obj.whole_body.w_qdd(r.findPositionIndices('back_bkx')) = 0.001;
      obj.whole_body.w_qdd(r.findPositionIndices('leg')) = 1e-6;

      obj.body_motion(r.foot_body_id.right).Kp = 48*ones(6,1);
      obj.body_motion(r.foot_body_id.right).damping_ratio = 0.8;
      obj.body_motion(r.foot_body_id.right).weight = 0.15;
      obj.body_motion(r.foot_body_id.left).Kp = 48*ones(6,1);
      obj.body_motion(r.foot_body_id.left).damping_ratio = 0.8;
      obj.body_motion(r.foot_body_id.left).weight = 0.15;
      obj.body_motion(r.findLinkId('pelvis')).Kp = [nan; nan; 20; 20; 20; 20];
      obj.body_motion(r.findLinkId('pelvis')).damping_ratio = 0.5;
      obj.body_motion(r.findLinkId('pelvis')).weight = 0.07;

      obj.Kp_accel = 0;

      % integral gains for position controlled joints
      integral_gains = zeros(getNumPositions(r),1);
      integral_clamps = zeros(getNumPositions(r),1);
      arm_ind = findPositionIndices(r,'arm');
      back_ind = findPositionIndices(r,'back');
      integral_gains(arm_ind) = 1.75;
      integral_gains(back_ind) = 0.3;
      integral_clamps(arm_ind) = 0.2;
      integral_clamps(back_ind) = 0.2;
      obj.whole_body.integrator.gains = integral_gains;
      obj.whole_body.integrator.clamps = integral_clamps;

      obj.vref_integrator.eta = 0.0;

      obj = obj.updateKd();

    end
  end
end
