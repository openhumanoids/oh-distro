function A = expm_taylor_l(F)
% need to do Pade corrections, but this is just to test

    A = eye(size(F)) + F + 0.5*F^2;
end