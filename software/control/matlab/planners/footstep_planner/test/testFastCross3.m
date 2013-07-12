function testFastCross3()

profile on
for j = 1:1000
  U = random('unif', -100, 100, 3, 1);
  V = random('unif', -100, 100, 3, 1);
  X = fastCross3(U, V);
  Y = cross(U, V);
  valuecheck(X, Y);
end
profile viewer