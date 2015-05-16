classdef PositionControlHardware < atlasParams.Base
  methods
    function obj = PositionControlHardware(r)
      typecheck(r, 'DRCAtlas');
      obj = obj@atlasParams.Base(r);

      force_controlled_joint_names = {};
      obj.hardware = drcAtlasParams.getHardwareParams(r, force_controlled_joint_names);


      % integral gains for position controlled joints
      integral_gains = zeros(getNumPositions(r),1);
      integral_clamps = zeros(getNumPositions(r),1);
      arm_ind = findPositionIndices(r,'arm');
      back_ind = findPositionIndices(r,'back');
      leg_ind = findPositionIndices(r,'leg');
      integral_gains(arm_ind) = 1.75; % TODO: generalize this
      integral_gains(back_ind) = 0.3;
      integral_gains(leg_ind) = 0.3;

      integral_clamps(arm_ind) = 0.1;
      integral_clamps(back_ind) = 0.1;
      integral_clamps(leg_ind) = 0.1;

      integral_eta = 0;
      
      obj.whole_body.integrator.gains = integral_gains;
      obj.whole_body.integrator.clamps = integral_clamps;
      obj.whole_body.integrator.eta = integral_eta;

      obj.vref_integrator.eta = 0.001;

      obj = obj.updateKd();

    end
  end
end

