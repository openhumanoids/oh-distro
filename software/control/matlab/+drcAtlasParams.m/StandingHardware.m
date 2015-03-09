classdef StandingHardware < atlasParams.Standing
  methods
    function obj = StandingHardware(r)
      typecheck(r, 'DRCAtlas');
      obj = obj@atlasParams.Standing(r);

      force_controlled_joint_names = {'leg', 'back_bkx'};
      obj.hardware_gains = drcAtlasParams.getHardwareGains(r, force_controlled_joint_names);

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
