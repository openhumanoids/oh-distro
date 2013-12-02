function Q = ladderInverseKinQ(r)
  cost = Point(r.getStateFrame,1);
  cost.base_x = 0;
  cost.base_y = 0;
  cost.base_z = 0;
  cost.base_roll = 1000;
  cost.base_pitch = 1000;
  cost.base_yaw = 0;
  cost.back_bkz = 1e2;
  cost.back_bky = 1e2;
  cost.back_bkx = 1e2;
  cost = double(cost);
  Q = diag(cost(1:r.getNumDOF));
end
