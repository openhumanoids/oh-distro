function yk = sim_generate_yk(camposek, trueX)
  xk = trueX;
  simerr = deg2rad(5);
  yk = [ atan((xk(1)-camposek(1))/xk(3)); atan((xk(2)-camposek(2))/xk(3))] + randn(2,1)*simerr;
end