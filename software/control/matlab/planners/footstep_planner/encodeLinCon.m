function msg = encodeLinCon(A, b)
  msg = drc.lin_con_t();
  msg.m = size(A, 1);
  msg.n = size(A, 2);
  msg.m_times_n = msg.m*msg.n;
  msg.A = reshape(A, [], 1);
  msg.b = b;
end
