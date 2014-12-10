
% test script to test C++ implementation of the matrix fraction computation
% used by lti_disc

F = [0 1; 0 0];
L = eye(2);
Q = [0.001 0; 0 0.005];
dt = 0.00333333;

%
  % Closed form integration of covariance
  % by matrix fraction decomposition
  %
  n   = size(F,1)
  Phi = [F L*Q*L'; zeros(n,n) -F']
  Phit = expm(Phi*dt)
  AB  = Phit*[zeros(n,n);eye(n)]
  AB_t  = expm(Phi*dt)*[zeros(n,n);eye(n)];
  
  over = AB(1:n,:)
  under = AB((n+1):(2*n),:)

  Q = over/under
  AB_t-AB;
  
  