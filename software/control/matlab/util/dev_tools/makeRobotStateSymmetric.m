function q_symmetric = makeRobotStateSymmetric(q,symmetry_mode)
  if nargin < 2
    symmetry_mode = 'average';
  end
  r_idx = [22 23 24 25 26 27 28 29 30 31 32 33]';
  l_idx = [10 11 12 13 14 15 16 17 18 19 20 21]';
  back_idx = [7,9];

  r_flip = [1; -1; 1; -1; 1; -1; -1; 1; 1; 1; -1; -1];
  q_symmetric = q;
  q_symmetric(back_idx) = 0;
  switch symmetry_mode
    case 'left'
      sym_states = q_symmetric(l_idx);
      q_symmetric(r_idx) = r_flip.*sym_states;
    case 'right'
      sym_states = q_symmetric(r_idx);
      q_symmetric(l_idx) = r_flip.*sym_states;
    case 'average'
      sym_states = (q_symmetric(l_idx) + r_flip.*q_symmetric(r_idx))/2;
      q_symmetric(l_idx) = sym_states;
      q_symmetric(r_idx) = r_flip.*sym_states;
  end
end
