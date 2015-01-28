function polytope = decodeLinCon(msg)
  A = reshape(msg.A, msg.m, msg.n);
  b = reshape(msg.b, msg.m, 1);
  polytope = iris.Polytope(A, b);
end