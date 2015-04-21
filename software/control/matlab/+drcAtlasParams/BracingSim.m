classdef BracingSim < atlasParams.Base
  methods
    function obj = BracingSim(r)
      typecheck(r, 'DRCAtlas');
      obj = obj@atlasParams.Base(r);

      obj.hardware = drcAtlasParams.getBracingParams(r);

      obj.whole_body.Kp = 110*ones(r.getNumPositions(), 1);
      obj.whole_body.Kp(1:6) = zeros(6, 1);
      obj.whole_body.damping_ratio = 1.2;

      obj.whole_body.w_qdd = ones(r.getNumVelocities(), 1);

      nbod = r.getManipulator().getNumBodies();
      obj.body_motion = struct('Kp', mat2cell(zeros(6, nbod), 6, ones(1, nbod)),...
                               'damping_ratio', 0.0,...
                               'accel_bounds', struct('min', [-100;-100;-100;-50;-50;-50],...
                                                      'max', [100;100;100;50;50;50]),...
                               'weight', num2cell(zeros(1, nbod)),...
                               'Kd', mat2cell(zeros(6, nbod), 6, ones(1, nbod)));
                                     
      % integral gains for position controlled joints
      integral_gains = zeros(getNumPositions(r),1);
      integral_clamps = zeros(getNumPositions(r),1);
      obj.whole_body.integrator.gains = integral_gains;
      obj.whole_body.integrator.clamps = integral_clamps;

      obj = obj.updateKd();
    end
  end
end
