classdef WalkingHardware < atlasParams.Walking
  methods
    function obj = WalkingHardware(r)
      typecheck(r, 'DRCAtlas');
      obj = obj@atlasParams.Walking(r);

      force_controlled_joint_names = {'leg', 'back_bkx'};
      obj.hardware = drcAtlasParams.getHardwareParams(r, force_controlled_joint_names);

      obj.whole_body.Kp = zeros(r.getNumPositions(), 1);
      obj.whole_body.Kp(r.findPositionIndices('back_bkx')) = 50;
      obj.whole_body.damping_ratio = 0.5;
      obj.whole_body.w_qdd = zeros(r.getNumVelocities(), 1);
      if (r.getNumVelocities() ~= r.getNumPositions())
        error('this code calls findPositionIndices, which is no longer equivalent to findVelocityIndices');
      end
      obj.whole_body.w_qdd(r.findPositionIndices('back_bkx')) = 0.01;

      obj.body_motion(r.foot_body_id.right).Kp = [12; 12; 12; 12; 12; 12];
      obj.body_motion(r.foot_body_id.right).damping_ratio = 0.7;
      obj.body_motion(r.foot_body_id.left).Kp = [12; 12; 12; 12; 12; 12];
      obj.body_motion(r.foot_body_id.left).damping_ratio = 0.7;

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
